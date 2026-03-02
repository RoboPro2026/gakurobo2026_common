/**
 * @file sabacan_robstride_node.cpp
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief robstrideのCANノード
 * @version 0.1
 * @date 2026-03-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <chrono>
#include <complex>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan/sabacan.h"
#include "sabacan_msgs/msg/sabacan_robstride_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robstride_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_msgs/srv/set_robstride_gains.hpp"

using namespace std::chrono_literals;

class SabacanRobstrideNode : public rclcpp::Node
{
public:
  explicit SabacanRobstrideNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sabacan_robstride_node", options)  // options を基底クラスに渡す
  {
    // パラメータの宣言

    // board_idを必須パラメータとして宣言（デフォルト値なし）
    // パラメータの制約や説明を記述子で定義
    auto board_id_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    board_id_descriptor.description =
      "The unique ID of the CAN board (0-255). This parameter is mandatory.";
    board_id_descriptor.integer_range.resize(1);
    board_id_descriptor.integer_range[0].from_value = 0;
    board_id_descriptor.integer_range[0].to_value = 255;
    board_id_descriptor.integer_range[0].step = 1;
    this->declare_parameter<int64_t>("board_id", board_id_descriptor);

    // 必須パラメータ'board_id'が設定されているかチェック
    try {
      this->get_parameter("board_id", board_id_);
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      // パラメータが設定されていなかった場合、致命的なエラーを出して終了
      RCLCPP_FATAL(
        this->get_logger(),
        "Mandatory parameter 'board_id' was not set. Please provide it at launch.\n"
        "Example: ros2 run your_package_name sabacan_robstride_node --ros-args -p "
        "board_id:=<value>");
      // コンストラクタで例外を投げることで、ノードの初期化を安全に中止します
      throw;
    }

    this->declare_parameter<int64_t>("can_master_id", 253);
    this->get_parameter("can_master_id", can_master_id_);

    this->declare_parameter("enable_initialize", true);
    this->get_parameter("enable_initialize", enable_initialize_);

    // robstride_typeの文字列をチェック
    this->declare_parameter("robstride_type", "RS05");
    this->get_parameter("robstride_type", robstride_type_str_);
    if (robstride_type_str_ == "RS05") {
      robstride_type_ = RobstrideType::RS05;
    } else if (robstride_type_str_ == "EL05") {
      robstride_type_ = RobstrideType::EL05;
    } else {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid robstride_type: %s. Valid options are 'RS05' and 'EL05'.",
        robstride_type_str_.c_str());
      return;
    }

    // TODO: 初期値はrobstride_type_によって変える
    this->declare_parameter("velocity_mode_limit_cur", 11.0f);
    this->declare_parameter("velocity_mode_acc_rad", 20.0f);
    this->declare_parameter("csp_mode_limit_spd", 50.0f);
    this->declare_parameter("pp_mode_vel_max", 10.0f);
    this->declare_parameter("pp_mode_acc_set", 10.0f);
    this->declare_parameter<int>("epscan_time_ms", 10);

    can_driver_ = std::make_shared<CanDriver>();
    robstride_driver_ =
      std::make_shared<RobstrideDriver>(can_driver_, board_id_, can_master_id_, robstride_type_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

    // CANデータ受信用のSubscription
    can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100,
      std::bind(&SabacanRobstrideNode::can_callback, this, std::placeholders::_1));

    // 現在のモータの状態を送信するPublisher
    sabacan_status_publisher_ = this->create_publisher<sabacan_msgs::msg::SabacanRobstrideStatus>(
      "/sabacan_robstride_status" + std::to_string(board_id_), 100);

    // モータの指令値を受け取るSubscription
    sabacan_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanRobstrideRef>(
      "/sabacan_robstride_ref" + std::to_string(board_id_), 100,
      std::bind(&SabacanRobstrideNode::ref_callback, this, std::placeholders::_1));

    // リセットサービスサーバーの作成
    reset_service_ = this->create_service<sabacan_msgs::srv::SabacanReset>(
      "sabacan_robstride_reset",
      std::bind(
        &SabacanRobstrideNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ゲイン設定サービスサーバーの作成
    set_gains_service_ = this->create_service<sabacan_msgs::srv::SetRobstrideGains>(
      "set_robstride_gains", std::bind(
                               &SabacanRobstrideNode::set_gains_callback, this,
                               std::placeholders::_1, std::placeholders::_2));

    // パラメータ変更コールバックの設定
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SabacanRobstrideNode::parameter_callback, this, std::placeholders::_1));

    // 100Hzでpublish用のtimerを呼ぶ
    publish_timer_ =
      this->create_wall_timer(10ms, std::bind(&SabacanRobstrideNode::publish_timer_callback, this));

    // 初期化命令を送信
    if (enable_initialize_) {
      robstride_init();
    }
  }

  template <typename T>
  bool check_size(std::vector<T> data, size_t _N, std::string name)
  {
    if (data.size() != _N) {
      RCLCPP_ERROR(this->get_logger(), "%s size error: size=%ld", name.c_str(), data.size());
      return false;
    }
    return true;
  }

  // データが範囲内かを確認
  template <typename T>
  bool check_data_range(T data, T low, T high, std::string name)
  {
    if ((low <= data && data <= high) == false) {
      if constexpr (std::is_integral<T>())
        if constexpr (sizeof(T) <= 4)
          RCLCPP_ERROR(this->get_logger(), "%s is out of range. value = %d", name.c_str(), data);
        else
          RCLCPP_ERROR(this->get_logger(), "%s is out of range. value = %ld", name.c_str(), data);
      else
        RCLCPP_ERROR(this->get_logger(), "%s is out of range. value = %f", name.c_str(), data);
      return false;
    }
    return true;
  }

  template <typename T>
  bool check_data_range_and_size(std::vector<T> data, T low, T high, size_t _N, std::string name)
  {
    if (check_size<T>(data, _N, name) == false) return false;
    for (size_t i = 0; i < _N; i++) {
      if (check_data_range<T>(data[i], low, high, name) == false) return false;
    }
    return true;
  }

  /**
   * @brief CanDriver用の送信コールバック関数
   *
   * @param id
   * @param data
   * @param dlc
   * @param is_remote_frame
   * @param is_ext_id
   */
  void tx(uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote_frame, bool is_ext_id = true)
  {
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    msg->header.stamp = this->get_clock()->now();
    msg->id = id;
    msg->is_extended = is_ext_id;
    msg->is_rtr = is_remote_frame;
    msg->dlc = dlc;
    for (int i = 0; i < 8; i++) {
      msg->data[i] = data[i];
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Sending CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]", msg->id,
      msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5],
      msg->data[6], msg->data[7]);
    can_publisher_->publish(std::move(msg));
  }

  void update_angle()
  {
    // 角度の積分
    // 面倒なので、積分するときは-pi~piの範囲に丸める
    raw_current_angle_ = robstride_driver_->angle;
    if (raw_current_angle_ - raw_prev_angle_ > 4 * M_PI) {
      // 角度が負から正に変化（負の方向に増加）
      turn_cnt_--;
    } else if (raw_current_angle_ - raw_prev_angle_ < -4 * M_PI) {
      // 角度が正から負に変化（正の方向に増加）
      turn_cnt_++;
    }
    integrated_current_angle_ = 8 * M_PI * turn_cnt_ + raw_current_angle_;
    // 前回値を更新
    raw_prev_angle_ = raw_current_angle_;
  }

  /**
   * @brief CAN受信時のcallback
   *
   * @param msg
   */
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
      msg->id, msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
      msg->data[5], msg->data[6], msg->data[7]);
    bool is_receive = robstride_driver_->receive(
      msg->id, msg->data.data(), msg->dlc, msg->is_rtr, msg->is_extended);
    if (is_receive) {
      update_angle();
    }
  }

  void ref_callback(const sabacan_msgs::msg::SabacanRobstrideRef::SharedPtr msg)
  {
    // 指令値のcontrol_typeが正しいか確認
    if (
      msg->control_type != "MIT" && msg->control_type != "CURRENT" &&
      msg->control_type != "VELOCITY" && msg->control_type != "PP" && msg->control_type != "CSP") {
      RCLCPP_ERROR(this->get_logger(), "Invalid control_type: %s.", msg->control_type.c_str());
      return;
    }

    // 前回とモードが変わっていたときは、モード切り替えの処理をする
    if (msg->control_type != prev_control_type_) {
      if (msg->control_type == "MIT") {
        robstride_driver_->setSingleParameterWrite_uint8(RobstrideIndex::RUN_MODE, 0x0);
      } else if (msg->control_type == "PP") {
        robstride_driver_->setSingleParameterWrite_uint8(RobstrideIndex::RUN_MODE, 0x1);
      } else if (msg->control_type == "VELOCITY") {
        robstride_driver_->setSingleParameterWrite_uint8(RobstrideIndex::RUN_MODE, 0x2);
      } else if (msg->control_type == "CURRENT") {
        robstride_driver_->setSingleParameterWrite_uint8(RobstrideIndex::RUN_MODE, 0x3);
      } else if (msg->control_type == "CSP") {
        robstride_driver_->setSingleParameterWrite_uint8(RobstrideIndex::RUN_MODE, 0x5);
      }
      // 念の為モータをenableにする
      robstride_driver_->setMotorEnabledToRun();
      RCLCPP_INFO(this->get_logger(), "Control mode changed: %s", msg->control_type.c_str());
    }

    // 指令値の送信
    if (msg->control_type == "MIT") {
      robstride_driver_->setOperationControlMode_MotorControlInstruction(
        msg->mit_torque, msg->mit_pos, msg->mit_speed, msg->mit_kp, msg->mit_kd);
      RCLCPP_INFO(
        this->get_logger(), "MIT control: torque=%f, pos=%f, speed=%f, kp=%f, kd=%f",
        msg->mit_torque, msg->mit_pos, msg->mit_speed, msg->mit_kp, msg->mit_kd);
    } else if (msg->control_type == "CURRENT") {
      robstride_driver_->setSingleParameterWrite_float(RobstrideIndex::IQ_REF, msg->current_ref);
      RCLCPP_INFO(this->get_logger(), "Current control: current_ref=%f", msg->current_ref);
    } else if (msg->control_type == "VELOCITY") {
      robstride_driver_->setSingleParameterWrite_float(RobstrideIndex::SPD_REF, msg->velocity_ref);
      RCLCPP_INFO(this->get_logger(), "Velocity control: velocity_ref=%f", msg->velocity_ref);
    } else if (msg->control_type == "PP") {
      robstride_driver_->setSingleParameterWrite_float(RobstrideIndex::LOC_REF, msg->pp_angle_ref);
      RCLCPP_INFO(this->get_logger(), "PP control: pp_angle_ref=%f", msg->pp_angle_ref);
    } else if (msg->control_type == "CSP") {
      robstride_driver_->setSingleParameterWrite_float(RobstrideIndex::LOC_REF, msg->csp_angle_ref);
      RCLCPP_INFO(this->get_logger(), "CSP control: csp_angle_ref=%f", msg->csp_angle_ref);
    }

    // 前回の値を更新
    prev_control_type_ = msg->control_type;
  }

  void publish_timer_callback()
  {
    // モータの状態をpublishする
    sabacan_msgs::msg::SabacanRobstrideStatus status_msg;
    status_msg.torque = robstride_driver_->torque;
    status_msg.speed = robstride_driver_->speed;
    status_msg.pos = integrated_current_angle_;
    sabacan_status_publisher_->publish(status_msg);
    RCLCPP_INFO(
      this->get_logger(), "Publishing status: torque=%f, speed=%f, pos=%f", status_msg.torque,
      status_msg.speed, status_msg.pos);
  }

  // パラメータ変更コールバック
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool ret = true;

    for (const auto & parameter : parameters) {
      if ((ret = update_parameters(parameter)))
        RCLCPP_INFO(this->get_logger(), "Updated parameter: %s", parameter.get_name().c_str());
      if (ret == false) {
        result.successful = false;
      }
    }

    return result;
  }

  void reset_callback(
    const std::shared_ptr<sabacan_msgs::srv::SabacanReset::Request> request,
    std::shared_ptr<sabacan_msgs::srv::SabacanReset::Response> response)
  {
    (void)request;
    robstride_init();
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Sabacan robstride reset successfully.");
  }

  // ゲイン設定サービスのコールバック
  void set_gains_callback(
    const std::shared_ptr<sabacan_msgs::srv::SetRobstrideGains::Request> request,
    std::shared_ptr<sabacan_msgs::srv::SetRobstrideGains::Response> response)
  {
    if (request->set_limit_cur) {
      this->set_parameter(rclcpp::Parameter("velocity_mode_limit_cur", request->limit_cur));
      robstride_driver_->setSingleParameterWrite_float(
        RobstrideIndex::LIMIT_CUR, request->limit_cur);
      RCLCPP_INFO(this->get_logger(), "Set limit_cur to %f", request->limit_cur);
    }
    response->success = true;
    response->message = "ok";
  }

  void delay() { std::this_thread::sleep_for(10ms); }

  void robstride_init()
  {
    // clang-format off
    std::vector<std::string> param_name{
      "velocity_mode_limit_cur",
      "velocity_mode_acc_rad",
      "csp_mode_limit_spd",
      "pp_mode_vel_max",
      "pp_mode_acc_set",
      "epscan_time_ms"
    };
    // clang-format on

    // 変数を初期化
    control_type_ = "";
    prev_control_type_ = "";
    raw_current_angle_ = 0;
    raw_prev_angle_ = 0;
    integrated_current_angle_ = 0.0f;
    turn_cnt_ = 0;
    // 安全のためモータを停止
    robstride_driver_->setMotorStopsRunning(1);
    delay();
    rclcpp::Time start_time = this->get_clock()->now();
    for (size_t i = 0; i < param_name.size(); i++) {
      // update_parameters関数内ではdelayが入っている。
      update_parameters(this->get_parameter(param_name[i]));
    }
    // フィードバックを有効
    robstride_driver_->setMotorActivelyReportsFrames(true);
    delay();
    // モータをスタート
    robstride_driver_->setMotorEnabledToRun();
    delay();

    rclcpp::Time end_time = this->get_clock()->now();
    RCLCPP_INFO(
      this->get_logger(), "Robstride initialization completed in %.3f seconds",
      (end_time - start_time).seconds());
  }

  bool update_parameters(rclcpp::Parameter parameter)
  {
    if (parameter.get_name() == "velocity_mode_limit_cur") {
      velocity_mode_limit_cur_ = parameter.as_double();
      robstride_driver_->setSingleParameterWrite_float(
        RobstrideIndex::LIMIT_CUR, velocity_mode_limit_cur_);
      delay();
    } else if (parameter.get_name() == "velocity_mode_acc_rad") {
      velocity_mode_acc_rad_ = parameter.as_double();
      robstride_driver_->setSingleParameterWrite_float(
        RobstrideIndex::ACC_RAD, velocity_mode_acc_rad_);
      delay();
    } else if (parameter.get_name() == "csp_mode_limit_spd") {
      csp_mode_limit_spd_ = parameter.as_double();
      robstride_driver_->setSingleParameterWrite_float(
        RobstrideIndex::LIMIT_SPD, csp_mode_limit_spd_);
      delay();
    } else if (parameter.get_name() == "pp_mode_vel_max") {
      pp_mode_vel_max_ = parameter.as_double();
      robstride_driver_->setSingleParameterWrite_float(RobstrideIndex::VEL_MAX, pp_mode_vel_max_);
      delay();
    } else if (parameter.get_name() == "pp_mode_acc_set") {
      pp_mode_acc_set_ = parameter.as_double();
      robstride_driver_->setSingleParameterWrite_float(RobstrideIndex::ACC_SET, pp_mode_acc_set_);
      delay();
    } else if (parameter.get_name() == "epscan_time_ms") {
      // epscan_timeの初期値は10msで1
      // 5ms増えるとepscan_timeが1増えるようにする
      epscan_time_ = (parameter.as_int() - 5) / 5;
      robstride_driver_->setSingleParameterWrite_uint16(RobstrideIndex::EPSCAN_TIME, epscan_time_);
      delay();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown parameter: %s", parameter.get_name().c_str());
      return false;
    }
    return true;
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobstrideRef>::SharedPtr sabacan_ref_subscription_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanRobstrideStatus>::SharedPtr sabacan_status_publisher_;
  rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
  rclcpp::Service<sabacan_msgs::srv::SetRobstrideGains>::SharedPtr set_gains_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<CanDriver> can_driver_;
  std::shared_ptr<RobstrideDriver> robstride_driver_;
  sabacan_msgs::msg::SabacanRobstrideRef ref_msg_;

  int64_t board_id_;
  int64_t can_master_id_;
  std::string robstride_type_str_;
  RobstrideType robstride_type_;
  bool enable_initialize_;
  // velocity modeのパラメータ
  float velocity_mode_limit_cur_;
  float velocity_mode_acc_rad_;
  // location mode(CSP)のパラメータ
  float csp_mode_limit_spd_;
  // location mode(PP)のパラメータ
  float pp_mode_vel_max_;
  float pp_mode_acc_set_;
  uint16_t epscan_time_;

  std::string control_type_;
  std::string prev_control_type_ = "";

  float raw_current_angle_ = 0;
  float raw_prev_angle_ = 0;
  float integrated_current_angle_ = 0.0f;
  int turn_cnt_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<SabacanRobstrideNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
