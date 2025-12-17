#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sabacan_msgs/msg/sabacan_robomas_ref.hpp>
#include <sabacan_msgs/msg/sabacan_robomas_status.hpp>
#include <sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

/*
メモ
- board_idとmotor_numberを指定して単体のモータを制御するノード
- トピック名でboard_idとmotor_numberを区別する
- publisherでsabacan_robomas_refトピックをpublishする
- subscriberで制御方式、指令値を受け取る
- parameterで基板側で制御するのか、ros2側で制御するのか選択可能にする。
その場合はsabacanパッケージ起動時に制御方式を"CURRENT"に設定する
- ros2側で制御するときはこのノードで速度制御、位置制御を可能にする
*/

class SabacanSingleControlNode : public rclcpp::Node
{
public:
  SabacanSingleControlNode() : Node("sabacan_single_control_node")
  {
    // パラメータ宣言
    this->declare_parameter("board_id", 0);
    this->declare_parameter("motor_number", 0);
    this->declare_parameter("control_mode", std::string("SABANE"));  // 制御モード(ROS, SABANE)
    this->declare_parameter("control_cycle", 100.0);                 // 制御周期[Hz]
    this->declare_parameter(
      "control_type", std::string("VELOCITY"));  // 制御方式(VELOCITY, POSITION, CURRENT, TORQUE)
    this->declare_parameter("change_mode_delay", 0.2);  // モード変更後の待機時間[s]

    board_id_ = this->get_parameter("board_id").as_int();
    motor_number_ = this->get_parameter("motor_number").as_int();
    control_mode_ = this->get_parameter("control_mode").as_string();
    control_type_ = this->get_parameter("control_type").as_string();
    float control_cycle_ = this->get_parameter("control_cycle").as_double();
    change_mode_delay_ = this->get_parameter("change_mode_delay").as_double();

    // ROSで制御する場合のフィードバックの取得
    sabacan_robomas_sub_ = this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
      "sabacan_robomas_status" + std::to_string(board_id_), 1,
      std::bind(
        &SabacanSingleControlNode::sabacanRobomasStatusCallback, this, std::placeholders::_1));

    // モータ単体の指令値と制御方式の取得
    sabacan_robomas_single_sub_ =
      this->create_subscription<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "sabacan_robomas_ref" + std::to_string(board_id_) + "/motor" +
          std::to_string(motor_number_),
        1,
        std::bind(
          &SabacanSingleControlNode::sabacanRobomasSingleRefCallback, this, std::placeholders::_1));

    // Sabacanにパブリッシュ
    sabacan_robomas_pub_ = this->create_publisher<sabacan_msgs::msg::SabacanRobomasRef>(
      "sabacan_robomas_ref" + std::to_string(board_id_), 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_cycle_),
      std::bind(&SabacanSingleControlNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "sabacan_single_control_node initialized");
    RCLCPP_INFO(this->get_logger(), "board_id: %d", board_id_);
    RCLCPP_INFO(this->get_logger(), "motor_number: %d", motor_number_);
    RCLCPP_INFO(this->get_logger(), "control_mode: %s", control_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "control_type: %s", control_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "control_type: %s", control_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "control_cycle: %f", control_cycle_);

    // 時刻比較での例外(Clock type mismatch)を防ぐため、ノードのクロックタイプで初期化
    control_mode_enable_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

private:
  void sabacanRobomasStatusCallback(const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    if (msg->motor_number == motor_number_) {
      sabacan_robomas_status_ = *msg;
    }
    // RCLCPP_DEBUG(this->get_logger(), "sabacan_robomas_status received");
    // RCLCPP_DEBUG(this->get_logger(), "motor_number: %d", msg->motor_number);
    // RCLCPP_DEBUG(this->get_logger(), "position: %f", msg->pos);
    // RCLCPP_DEBUG(this->get_logger(), "velocity: %f", msg->speed);
    // RCLCPP_DEBUG(this->get_logger(), "torque: %f", msg->torque);
  }

  void sabacanRobomasSingleRefCallback(
    const sabacan_single_control_msgs::msg::SabacanRobomasSingleRef::SharedPtr msg)
  {
    sabacan_robomas_single_ref_ = *msg;
    // RCLCPP_DEBUG(this->get_logger(), "sabacan_robomas_single_ref received");
    // RCLCPP_DEBUG(this->get_logger(), "control_type: %s", msg->control_type.c_str());
    // RCLCPP_DEBUG(this->get_logger(), "ref: %f", msg->ref);
  }

  void timerCallback()
  {
    if (control_mode_ == "ROS") {
      // ROS側で制御
      motor_control_ros();
    } else if (control_mode_ == "SABANE") {
      // 基板側で制御
      if (sabacan_robomas_single_ref_.control_type != prev_control_type_) {
        if (!is_pre_switch_safe_sent_) {
          // 制御方式変更前に、現在のモードに応じた安全な指令値を1周期だけ送信する
          // 多分これで、モード切り替え時に暴走する問題は解決すると思う
          float safe_ref = 0.0;
          if (prev_control_type_ == "POSITION") { // 前の制御方式が位置制御の場合はフィードバックから得た現在の値を送信
            safe_ref = sabacan_robomas_status_.pos;
          } else { // 速度制御、トルク制御のときは0.0を送信する
            safe_ref = 0.0;  // 停止
          }

          auto msg = sabacan_msgs::msg::SabacanRobomasRef();
          msg.motor_number = motor_number_;
          msg.ref = safe_ref;
          sabacan_robomas_pub_->publish(msg);

          is_pre_switch_safe_sent_ = true;
          RCLCPP_INFO(
            this->get_logger(),
            "Pre-switch safety command sent (type=%s, ref=%f). Switching next cycle.",
            prev_control_type_.c_str(), safe_ref);
          return;
        }

        // 制御方式が変わったときのみ、パラメータを変更する
        setSingleRobomasControlType(sabacan_robomas_single_ref_.control_type);
        prev_control_type_ = sabacan_robomas_single_ref_.control_type;
        control_type_ = sabacan_robomas_single_ref_.control_type;
        is_pre_switch_safe_sent_ = false;
      }
      // control_type の変更中、または変更後の待機時間中はpublishしない
      if (pending_control_type_change_ || this->now() < control_mode_enable_time_) {
        // 何もしない (ref=0.0 も送らない)
        // pending_control_type_change_ : パラメータ変更の応答待ち
        // control_mode_enable_time_    : 変更後の安定化待機時間
      } else {
        auto msg = sabacan_msgs::msg::SabacanRobomasRef();
        msg.motor_number = motor_number_;
        msg.ref = sabacan_robomas_single_ref_.ref;
        sabacan_robomas_pub_->publish(msg);
      }
    }
    RCLCPP_INFO(this->get_logger(), "control_mode: %s", control_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "control_type: %s", control_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "ref: %f", sabacan_robomas_single_ref_.ref);
  }

  void motor_control_ros()
  {
    // そのうち実装する
  }

  // sabacan_robomas_node<board_id> の control_type パラメータを設定する
  // CLI の `ros2 param set /sabacan_robomas_node<board_id> control_type [...]` と等価
  bool setRobomasControlType(const std::vector<std::string> & control_type)
  {
    // V1基板は4モーター想定。サイズ不一致はエラー扱い
    if (control_type.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(), "control_type must have 4 elements. given: %zu", control_type.size());
      return false;
    }

    const std::string service_name =
      "/sabacan_robomasv2_node_id" + std::to_string(board_id_) + "/set_parameters";
    // 永続クライアントを用意
    if (!set_param_client_) {
      set_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(service_name);
    }

    if (!set_param_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available: %s", service_name.c_str());
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter param;
    param.name = "control_type";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    param.value.string_array_value = control_type;
    request->parameters.push_back(param);

    // 非ブロッキングで送信。応答はコールバックでログのみ行う
    auto future = set_param_client_->async_send_request(
      request, [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture response) {
        bool all_success = true;
        for (const auto & result : response.get()->results) {
          if (!result.successful) {
            all_success = false;
            RCLCPP_WARN(this->get_logger(), "Parameter set failed: %s", result.reason.c_str());
          }
        }
        if (all_success) {
          // control_type 変更完了。Delayを設定してフラグ解除
          // ※ ここでフラグをfalseにするが、timerCallback側で時刻チェックも行っているため安全
          control_mode_enable_time_ = this->now() + rclcpp::Duration::from_seconds(change_mode_delay_);
          pending_control_type_change_ = false;
          RCLCPP_INFO(
            this->get_logger(), "Set control_type on /sabacan_robomasv2_node_id%d successfully. Wait %.2fs",
            board_id_, change_mode_delay_);
        }
      });
    (void)future;  // 使わないが、ここで破棄しない
    return true;
  }

  // 単一モータの control_type を変更（V2仕様の値を許容）
  bool setSingleRobomasControlType(const std::string & control_type)
  {
    if (motor_number_ > 3) {
      RCLCPP_WARN(this->get_logger(), "motor_number must be 0..3. given: %u", motor_number_);
      return false;
    }

    // 許容されるV2の制御モード
    static const std::unordered_set<std::string> kAllowed = {"TORQUE", "VELOCITY", "POSITION",
                                                             "PWM",    "CURRENT",  "DISABLE"};
    if (kAllowed.find(control_type) == kAllowed.end()) {
      RCLCPP_WARN(this->get_logger(), "Unsupported control_type for V2: %s", control_type.c_str());
      return false;
    }

    // 変更処理中フラグを立てる（応答成功時に解除）
    pending_control_type_change_ = true;

    const std::string get_srv =
      "/sabacan_robomasv2_node_id" + std::to_string(board_id_) + "/get_parameters";
    if (!get_param_client_) {
      get_param_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(get_srv);
    }
    if (!get_param_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available: %s", get_srv.c_str());
      return false;
    }

    auto get_req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_req->names = {"control_type"};
    // 非ブロッキング: get -> set を応答コールバックで連鎖
    auto future_get = get_param_client_->async_send_request(
      get_req, [this, control_type](
                 rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture get_res_fut) {
        auto get_res = get_res_fut.get();
        if (get_res->values.size() != 1) {
          RCLCPP_ERROR(
            this->get_logger(), "get_parameters returned unexpected number of values: %zu",
            get_res->values.size());
          return;
        }
        std::vector<std::string> array = {"DISABLE", "DISABLE", "DISABLE", "DISABLE"};
        const auto & pv = get_res->values[0];
        if (pv.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) {
          const auto & arr = pv.string_array_value;
          if (arr.size() == 4) {
            array = arr;
          } else {
            RCLCPP_WARN(
              this->get_logger(),
              "control_type array size is %zu. Using defaults and overriding target index.",
              arr.size());
          }
        } else if (pv.type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
          RCLCPP_WARN(
            this->get_logger(), "control_type param type is not STRING_ARRAY. type=%u", pv.type);
        }
        array[motor_number_] = control_type;
        (void)setRobomasControlType(array);
        RCLCPP_INFO(
          this->get_logger(), "Set control_type[%u]=%s requested on /sabacan_robomasv2_node_id%d",
          motor_number_, control_type.c_str(), board_id_);

        // パラメータ設定リクエストを投げたので、完了応答待ちを解除するが、
        // 実際にはsetRobomasControlTypeの非同期応答(set_param_client_)でpending解除すべき。
        // しかし現在の構造ではsetRobomasControlType内ですぐにpending解除しているため、
        // ここでは追加のDelayを設定する。
        // ※ 本来は setRobomasControlType のコールバックチェーンでフラグ操作すべきだが、
        //    既存コード(setRobomasControlType)がpending_control_type_change_を操作しているのでそれに従う。
        //    ただし、setRobomasControlTypeの実装を見ると、全てのresultがsuccessのときのみフラグを下ろしている。
        //    ここ(get_param callback)はリクエストを投げただけなので、まだフラグを下ろしてはいけない。
        //    setRobomasControlType側のcallbackでフラグが落ち、そのタイミングでDelayをセットする必要がある。
      });
    (void)future_get;
    return true;
  }

  int board_id_;
  int motor_number_;
  std::string control_mode_;
  std::string control_type_;
  std::string
    prev_control_type_;  // 前回の制御方式（制御方式が変わったときのみ、パラメータを変更する）

  rclcpp::Subscription<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    sabacan_robomas_single_sub_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr sabacan_robomas_sub_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr sabacan_robomas_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_param_client_;
  sabacan_msgs::msg::SabacanRobomasRef sabacan_robomas_ref_;
  sabacan_msgs::msg::SabacanRobomasStatus sabacan_robomas_status_;
  sabacan_single_control_msgs::msg::SabacanRobomasSingleRef sabacan_robomas_single_ref_;

  // control_type 変更完了まで ref を 0 に固定するためのフラグ
  bool pending_control_type_change_ = false;
  double change_mode_delay_;

  rclcpp::Time control_mode_enable_time_;
  bool is_pre_switch_safe_sent_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SabacanSingleControlNode>());
  rclcpp::shutdown();
  return 0;
}