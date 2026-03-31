#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sabacan_msgs/msg/sabacan_robomas_ref.hpp>
#include <sabacan_msgs/msg/sabacan_robomas_status.hpp>
#include <sabacan_msgs/srv/set_robomas_gains.hpp>
#include <sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp>
#include <algorithm>
#include <cctype>
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
    this->declare_parameter(
      "control_type_update_method",
      std::string("parameter"));  // control_type切替方法(parameter, service)

    board_id_ = this->get_parameter("board_id").as_int();
    motor_number_ = this->get_parameter("motor_number").as_int();
    control_mode_ = this->get_parameter("control_mode").as_string();
    control_type_ = this->get_parameter("control_type").as_string();
    float control_cycle_ = this->get_parameter("control_cycle").as_double();
    change_mode_delay_ = this->get_parameter("change_mode_delay").as_double();
    control_type_update_method_ =
      normalize_control_type_update_method(
        this->get_parameter("control_type_update_method").as_string());

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
    RCLCPP_INFO(
      this->get_logger(), "control_type_update_method: %s", control_type_update_method_.c_str());
    if (control_type_update_method_ == "parameter") {
      RCLCPP_WARN(
        this->get_logger(),
        "control_type_update_method=parameter is kept for compatibility. service is recommended.");
    }

    // 時刻比較での例外(Clock type mismatch)を防ぐため、ノードのクロックタイプで初期化
    control_mode_enable_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

private:
  static std::string normalize_control_type_update_method(const std::string & method)
  {
    std::string normalized = method;
    std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "service") {
      return normalized;
    }
    return "parameter";
  }

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
          if (prev_control_type_ == "POSITION") {
            safe_ref = last_pos_ref_;  // 直前の位置指令を維持
          } else {
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

        // 位置制御時の指令値を保存
        if (control_type_ == "POSITION") {
          last_pos_ref_ = sabacan_robomas_single_ref_.ref;
        }
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

  bool setRobomasControlTypeByParameter(const std::vector<std::string> & control_type)
  {
    if (control_type.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(), "control_type must have 4 elements. given: %zu", control_type.size());
      return false;
    }

    const std::string service_name =
      "/sabacan_robomasv2_node_id" + std::to_string(board_id_) + "/set_parameters";
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

    auto future = set_param_client_->async_send_request(
      request, [this, control_type](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture response) {
        bool all_success = true;
        for (const auto & result : response.get()->results) {
          if (!result.successful) {
            all_success = false;
            RCLCPP_WARN(this->get_logger(), "Parameter set failed: %s", result.reason.c_str());
          }
        }
        if (!all_success) {
          pending_control_type_change_ = false;
          return;
        }

        prev_control_type_ = control_type[motor_number_];
        control_type_ = control_type[motor_number_];
        control_mode_enable_time_ = this->now() + rclcpp::Duration::from_seconds(change_mode_delay_);
        pending_control_type_change_ = false;
        RCLCPP_INFO(
          this->get_logger(),
          "Set control_type[%d]=%s successfully via parameter on board_id=%d. Wait %.2fs",
          motor_number_, control_type_.c_str(), board_id_, change_mode_delay_);
      });
    (void)future;
    return true;
  }

  bool setSingleRobomasControlTypeByParameter(const std::string & control_type)
  {
    static const std::unordered_set<std::string> kAllowed = {"TORQUE", "VELOCITY", "POSITION",
                                                             "PWM",    "CURRENT",  "DISABLE"};
    if (kAllowed.find(control_type) == kAllowed.end()) {
      RCLCPP_WARN(
        this->get_logger(), "Unsupported control_type for parameter mode: %s", control_type.c_str());
      return false;
    }

    const std::string get_srv =
      "/sabacan_robomasv2_node_id" + std::to_string(board_id_) + "/get_parameters";
    if (!get_param_client_) {
      get_param_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(get_srv);
    }
    if (!get_param_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available: %s", get_srv.c_str());
      pending_control_type_change_ = false;
      return false;
    }

    auto get_req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_req->names = {"control_type"};
    auto future_get = get_param_client_->async_send_request(
      get_req, [this, control_type](
                 rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture get_res_fut) {
        auto get_res = get_res_fut.get();
        if (get_res->values.size() != 1) {
          pending_control_type_change_ = false;
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
        if (!setRobomasControlTypeByParameter(array)) {
          pending_control_type_change_ = false;
          return;
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Set control_type[%d]=%s requested via parameter on /sabacan_robomasv2_node_id%d",
          motor_number_, control_type.c_str(), board_id_);
      });
    (void)future_get;
    return true;
  }

  bool setSingleRobomasControlType(const std::string & control_type)
  {
    if (motor_number_ > 3) {
      RCLCPP_WARN(this->get_logger(), "motor_number must be 0..3. given: %u", motor_number_);
      return false;
    }

    // 変更処理中フラグを立てる（応答成功時に解除）
    pending_control_type_change_ = true;

    if (control_type_update_method_ == "parameter") {
      return setSingleRobomasControlTypeByParameter(control_type);
    }

    const std::string service_name = "set_robomas_gains";
    if (!set_gains_client_) {
      set_gains_client_ = this->create_client<sabacan_msgs::srv::SetRobomasGains>(service_name);
    }
    if (!set_gains_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available: %s", service_name.c_str());
      pending_control_type_change_ = false;
      return false;
    }

    auto request = std::make_shared<sabacan_msgs::srv::SetRobomasGains::Request>();
    request->motor_number = static_cast<uint8_t>(motor_number_);
    request->control_type = control_type;
    request->set_control_type = true;

    auto future = set_gains_client_->async_send_request(
      request, [this, control_type](
                 rclcpp::Client<sabacan_msgs::srv::SetRobomasGains>::SharedFuture response) {
        const auto result = response.get();
        if (!result->success) {
          pending_control_type_change_ = false;
          RCLCPP_WARN(
            this->get_logger(), "Set control_type failed on board_id=%d motor=%d: %s", board_id_,
            motor_number_, result->message.c_str());
          return;
        }

        prev_control_type_ = control_type;
        control_type_ = control_type;
        control_mode_enable_time_ = this->now() + rclcpp::Duration::from_seconds(change_mode_delay_);
        pending_control_type_change_ = false;
        RCLCPP_INFO(
          this->get_logger(),
          "Set control_type[%d]=%s successfully on board_id=%d. Wait %.2fs", motor_number_,
          control_type.c_str(), board_id_, change_mode_delay_);
      });
    (void)future;
    return true;
  }

  int board_id_;
  int motor_number_;
  std::string control_mode_;
  std::string control_type_;
  std::string control_type_update_method_;
  std::string
    prev_control_type_;  // 前回の制御方式（制御方式が変わったときのみ、パラメータを変更する）

  rclcpp::Subscription<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    sabacan_robomas_single_sub_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr sabacan_robomas_sub_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr sabacan_robomas_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_param_client_;
  rclcpp::Client<sabacan_msgs::srv::SetRobomasGains>::SharedPtr set_gains_client_;
  sabacan_msgs::msg::SabacanRobomasRef sabacan_robomas_ref_;
  sabacan_msgs::msg::SabacanRobomasStatus sabacan_robomas_status_;
  sabacan_single_control_msgs::msg::SabacanRobomasSingleRef sabacan_robomas_single_ref_;

  // control_type 変更完了まで ref を 0 に固定するためのフラグ
  bool pending_control_type_change_ = false;
  double change_mode_delay_;

  rclcpp::Time control_mode_enable_time_;
  bool is_pre_switch_safe_sent_ = false;
  float last_pos_ref_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SabacanSingleControlNode>());
  rclcpp::shutdown();
  return 0;
}
