#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "sabacan/sabacan.h"
#include "sabacan_msgs/msg/sabacan_robomas_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_msgs/srv/set_robomas_gains.hpp"

using namespace std::chrono_literals;

class SabaneCanNode : public rclcpp::Node
{
public:
  SabaneCanNode() : Node("sabacan_robomas_node")
  {
    // パラメータの宣言
    this->declare_parameter("motor_type",
                            std::vector<std::string>{"Robomas", "Robomas", "Robomas", "Robomas"});
    this->declare_parameter(
        "control_type", std::vector<std::string>{"VELOCITY", "VELOCITY", "VELOCITY", "VELOCITY"});
    this->declare_parameter("board_id", uint8_t(0));

    // PIDゲインパラメータの宣言
    this->declare_parameter("speed_gain_p", std::vector<double>{0.5, 0.5, 0.5, 0.5});
    this->declare_parameter("speed_gain_i", std::vector<double>{0.2, 0.2, 0.2, 0.2});
    this->declare_parameter("speed_gain_d", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("pos_gain_p", std::vector<double>{6.0, 6.0, 6.0, 6.0});
    this->declare_parameter("pos_gain_i", std::vector<double>{3.0, 3.0, 3.0, 3.0});
    this->declare_parameter("pos_gain_d", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("speed_lim", std::vector<double>{30.0, 30.0, 30.0, 30.0});
    // モニタリング用パラメータ（V1でもV2同様に周期送信を有効化する）
    this->declare_parameter("monitor_period", 50);
    // モニタ周期設定の有効/無効を制御するパラメータ
    this->declare_parameter("enable_monitor_period", true); // デフォルトtrue
    int64_t reg = 0LL;
    reg |= 1LL << RobomasV1::MOTOR_STATE;
    reg |= 1LL << RobomasV1::PWM;
    reg |= 1LL << RobomasV1::SPEED;
    reg |= 1LL << RobomasV1::POSITION;
    reg |= 1LL << RobomasV1::POSITION_ABS;
    reg |= 1LL << RobomasV1::SPEED_ABS;
    reg |= 1LL << RobomasV1::ABS_TURN_CNT;
    this->declare_parameter("monitor_reg", std::vector<int64_t>{reg, reg, reg, reg});
    // パラメータ値の取得
    auto motor_type_param = this->get_parameter("motor_type").as_string_array();
    auto control_type_param = this->get_parameter("control_type").as_string_array();
    auto speed_gain_p_param = this->get_parameter("speed_gain_p").as_double_array();
    auto speed_gain_i_param = this->get_parameter("speed_gain_i").as_double_array();
    auto speed_gain_d_param = this->get_parameter("speed_gain_d").as_double_array();
    auto pos_gain_p_param = this->get_parameter("pos_gain_p").as_double_array();
    auto pos_gain_i_param = this->get_parameter("pos_gain_i").as_double_array();
    auto pos_gain_d_param = this->get_parameter("pos_gain_d").as_double_array();
    auto speed_lim_param = this->get_parameter("speed_lim").as_double_array();

    for (size_t i = 0; i < 4 && i < motor_type_param.size(); i++)
    {
      motor_type[i] = motor_type_param[i];
    }
    for (size_t i = 0; i < 4 && i < control_type_param.size(); i++)
    {
      control_type[i] = control_type_param[i];
    }
    for (size_t i = 0; i < 4 && i < speed_gain_p_param.size(); i++)
    {
      speed_gain_p_[i] = static_cast<float>(speed_gain_p_param[i]);
    }
    for (size_t i = 0; i < 4 && i < speed_gain_i_param.size(); i++)
    {
      speed_gain_i_[i] = static_cast<float>(speed_gain_i_param[i]);
    }
    for (size_t i = 0; i < 4 && i < speed_gain_d_param.size(); i++)
    {
      speed_gain_d_[i] = static_cast<float>(speed_gain_d_param[i]);
    }
    for (size_t i = 0; i < 4 && i < pos_gain_p_param.size(); i++)
    {
      pos_gain_p_[i] = static_cast<float>(pos_gain_p_param[i]);
    }
    for (size_t i = 0; i < 4 && i < pos_gain_i_param.size(); i++)
    {
      pos_gain_i_[i] = static_cast<float>(pos_gain_i_param[i]);
    }
    for (size_t i = 0; i < 4 && i < pos_gain_d_param.size(); i++)
    {
      pos_gain_d_[i] = static_cast<float>(pos_gain_d_param[i]);
    }
    for (size_t i = 0; i < 4 && i < speed_lim_param.size(); i++)
    {
      speed_lim_[i] = static_cast<float>(speed_lim_param[i]);
    }

    uint8_t board_id = this->get_parameter("board_id").as_int();
    enable_monitor_period_ = this->get_parameter("enable_monitor_period").as_bool();

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

    // CANデータ受信用のSubscriber
    can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 100, std::bind(&SabaneCanNode::can_callback, this, std::placeholders::_1));

    // 現在のモータ状態をpublishするPublisher
    sabacan_status_publisher_ = this->create_publisher<sabacan_msgs::msg::SabacanRobomasStatus>(
        "/sabacan_robomas_status" + std::to_string(board_id), 100);

    // モータの指令値を受け取るSubscriber
    sabacan_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanRobomasRef>(
        "/sabacan_robomas_ref" + std::to_string(board_id), 100,
        std::bind(&SabaneCanNode::sabacan_ref_callback, this, std::placeholders::_1));

    can_driver_ = std::make_shared<CanDriver>();

    robomas_driver_ = std::make_unique<RobomasDriverV1>(can_driver_, board_id);

    can_driver_->register_tx_callback(
        [this](uint32_t id, uint8_t* data, uint8_t dlc, bool is_remote, bool is_ext)
        { this->tx(id, data, dlc, is_remote, is_ext); });

    // サービスサーバーの作成
    set_gains_service_ = this->create_service<sabacan_msgs::srv::SetRobomasGains>(
        "set_robomas_gains", std::bind(&SabaneCanNode::set_gains_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    // リセットサービスサーバーの作成
    reset_service_ = this->create_service<sabacan_msgs::srv::SabacanReset>(
        "sabacan_robomas_reset", std::bind(&SabaneCanNode::reset_callback, this,
                                           std::placeholders::_1, std::placeholders::_2));

    // パラメータ変更コールバックの設定
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SabaneCanNode::parameter_callback, this, std::placeholders::_1));

    // 100Hzで状態publish用のタイマー
    publish_timer_ = this->create_wall_timer(
        10ms, std::bind(&SabaneCanNode::publish_timer_callback, this));

    robomas_init();
  }

  void tx(uint32_t id, uint8_t* data, uint8_t dlc, bool is_remote_frame, bool is_ext_id = true)
  {
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    msg->header.stamp = this->get_clock()->now();
    msg->id = id;
    msg->is_extended = is_ext_id;
    msg->is_rtr = is_remote_frame;
    msg->dlc = dlc;
    for (int i = 0; i < 8; i++)
    {
      msg->data[i] = data[i];
    }
    RCLCPP_INFO(this->get_logger(), "Sending CAN frame: ID=0x%X", msg->id);
    can_publisher_->publish(std::move(msg));
  }

private:
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg) const
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Received CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        msg->id, msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
        msg->data[5], msg->data[6], msg->data[7]);
    CanFrame frame;
    uint8_t data[8];
    for (int i = 0; i < 8; i++)
    {
      data[i] = msg->data[i];
    }
    frame = can_driver_->rx_frame(msg->id, data, msg->dlc, msg->is_rtr, msg->is_extended);
    robomas_driver_->receive(frame);
  }

  void timer_callback()
  {
    // gpio_driver_->
    // robomas_init();
  }

  void publish_timer_callback()
  {
    for (int i = 0; i < 4; i++)
    {
      auto msg = sabacan_msgs::msg::SabacanRobomasStatus();
      msg.motor_number = i;
      msg.motor_type = motor_type[i];
      msg.control_type = control_type[i];
      msg.motor_state = robomas_driver_->motor_state[i];
      // V1にはトルクの概念がないため、PWM相当値をtorqueに格納
      msg.torque = robomas_driver_->current_pwm[i];
      msg.speed = robomas_driver_->current_speed[i];
      msg.pos = robomas_driver_->current_pos[i];
      msg.abs_pos = robomas_driver_->current_abs_pos[i];
      msg.abs_speed = robomas_driver_->current_abs_speed[i];
      msg.abs_turn_cnt = robomas_driver_->abs_turn_cnt[i];
      sabacan_status_publisher_->publish(msg);
    }
  }

  void sabacan_ref_callback(const sabacan_msgs::msg::SabacanRobomasRef::SharedPtr msg)
  {
    if (control_type[msg->motor_number] == "VELOCITY")
    {
      robomas_driver_->setSpeedTarget(msg->motor_number, msg->ref);
    }
    else if (control_type[msg->motor_number] == "POSITION")
    {
      robomas_driver_->setPosTarget(msg->motor_number, msg->ref);
    }
    else if (control_type[msg->motor_number] == "OPEN")
    {
      robomas_driver_->setPwmTarget(msg->motor_number, msg->ref);
    }
    RCLCPP_INFO(this->get_logger(), "Received SabacanRef: motor_number=%d, ref=%f",
                msg->motor_number, msg->ref);
  }

  void robomas_init()
  {
    for (int i = 0; i < 4; i++)
    {
      if (motor_type[i] == "Robomas")
      {
        robomas_driver_->setMotorType(i, RobomasV1::MOTOR_TYPE_ROBOMAS);
      }
      else if (motor_type[i] == "VESC")
      {
        robomas_driver_->setMotorType(i, RobomasV1::MOTOR_TYPE_VESC);
      }
      if (control_type[i] == "VELOCITY")
      {
        robomas_driver_->setControlType(i, RobomasV1::CONTROL_TYPE_SPEED_MODE);
      }
      else if (control_type[i] == "POSITION")
      {
        robomas_driver_->setControlType(i, RobomasV1::CONTROL_TYPE_POSITION_MODE);
      }
      else if (control_type[i] == "OPEN")
      {
        robomas_driver_->setControlType(i, RobomasV1::CONTROL_TYPE_PWM_MODE);
      }

      // PIDゲインと速度制限の設定
      robomas_driver_->setSpeedGainP(i, speed_gain_p_[i]);
      robomas_driver_->setSpeedGainI(i, speed_gain_i_[i]);
      robomas_driver_->setSpeedGainD(i, speed_gain_d_[i]);
      robomas_driver_->setPosGainP(i, pos_gain_p_[i]);
      robomas_driver_->setPosGainI(i, pos_gain_i_[i]);
      robomas_driver_->setPosGainD(i, pos_gain_d_[i]);
      robomas_driver_->setSpeedLim(i, speed_lim_[i]);
    }

    // 監視設定（V1基板にも周期送信を依頼）
    int monitor_period = this->get_parameter("monitor_period").as_int();
    auto monitor_reg = this->get_parameter("monitor_reg").as_integer_array();
    for (size_t i = 0; i < 4 && i < monitor_reg.size(); i++)
        {
            robomas_driver_->setMonitorReg(i, static_cast<uint64_t>(monitor_reg[i]));
        }
    if (enable_monitor_period_) {
        robomas_driver_->setMonitorPeriod(static_cast<uint16_t>(monitor_period)); 
    }
  }

  // パラメータ変更コールバック
  rcl_interfaces::msg::SetParametersResult
  parameter_callback(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& parameter : parameters)
    {
      if (parameter.get_name().substr(0, 11) == "speed_gain_" ||
          parameter.get_name().substr(0, 9) == "pos_gain_" || parameter.get_name() == "speed_lim")
      {
        // PIDゲインまたは速度制限の更新
        update_gains_from_parameters();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: %s", parameter.get_name().c_str());
      }
    }

    return result;
  }

  // サービスコールバック
  void
  set_gains_callback(const std::shared_ptr<sabacan_msgs::srv::SetRobomasGains::Request> request,
                     std::shared_ptr<sabacan_msgs::srv::SetRobomasGains::Response> response)
  {
    if (request->motor_number >= 4)
    {
      response->success = false;
      response->message = "Invalid motor number. Must be 0-3.";
      return;
    }

    try
    {
      int motor_num = request->motor_number;

      if (request->set_speed_gains)
      {
        speed_gain_p_[motor_num] = request->speed_gain_p;
        speed_gain_i_[motor_num] = request->speed_gain_i;
        speed_gain_d_[motor_num] = request->speed_gain_d;

        robomas_driver_->setSpeedGainP(motor_num, speed_gain_p_[motor_num]);
        robomas_driver_->setSpeedGainI(motor_num, speed_gain_i_[motor_num]);
        robomas_driver_->setSpeedGainD(motor_num, speed_gain_d_[motor_num]);
      }

      if (request->set_pos_gains)
      {
        pos_gain_p_[motor_num] = request->pos_gain_p;
        pos_gain_i_[motor_num] = request->pos_gain_i;
        pos_gain_d_[motor_num] = request->pos_gain_d;

        robomas_driver_->setPosGainP(motor_num, pos_gain_p_[motor_num]);
        robomas_driver_->setPosGainI(motor_num, pos_gain_i_[motor_num]);
        robomas_driver_->setPosGainD(motor_num, pos_gain_d_[motor_num]);
      }

      if (request->set_speed_limit)
      {
        speed_lim_[motor_num] = request->speed_lim;
        robomas_driver_->setSpeedLim(motor_num, speed_lim_[motor_num]);
      }

      response->success = true;
      response->message = "Gains updated successfully for motor " + std::to_string(motor_num);

      RCLCPP_INFO(this->get_logger(), "Updated gains for motor %d via service", motor_num);
    }
    catch (const std::exception& e)
    {
      response->success = false;
      response->message = std::string("Error updating gains: ") + e.what();
    }
  }

  // リセットサービスコールバック
  void reset_callback(const std::shared_ptr<sabacan_msgs::srv::SabacanReset::Request> request,
                      std::shared_ptr<sabacan_msgs::srv::SabacanReset::Response> response)
  {
    (void)request; // 未使用パラメータ警告を抑制
    try
    {
      RCLCPP_INFO(this->get_logger(), "Received reset request for Robomas node");

      // Robomas初期化処理を実行
      robomas_init();

      response->success = true;
      response->message = "Robomas node reset completed successfully";

      RCLCPP_INFO(this->get_logger(), "Robomas node reset completed");
    }
    catch (const std::exception& e)
    {
      response->success = false;
      response->message = std::string("Error during Robomas reset: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Robomas reset failed: %s", e.what());
    }
  }

  // パラメータからゲインを更新する関数
  void update_gains_from_parameters()
  {
    auto speed_gain_p_param = this->get_parameter("speed_gain_p").as_double_array();
    auto speed_gain_i_param = this->get_parameter("speed_gain_i").as_double_array();
    auto speed_gain_d_param = this->get_parameter("speed_gain_d").as_double_array();
    auto pos_gain_p_param = this->get_parameter("pos_gain_p").as_double_array();
    auto pos_gain_i_param = this->get_parameter("pos_gain_i").as_double_array();
    auto pos_gain_d_param = this->get_parameter("pos_gain_d").as_double_array();
    auto speed_lim_param = this->get_parameter("speed_lim").as_double_array();

    for (size_t i = 0; i < 4; i++)
    {
      if (i < speed_gain_p_param.size())
      {
        speed_gain_p_[i] = static_cast<float>(speed_gain_p_param[i]);
        robomas_driver_->setSpeedGainP(i, speed_gain_p_[i]);
      }
      if (i < speed_gain_i_param.size())
      {
        speed_gain_i_[i] = static_cast<float>(speed_gain_i_param[i]);
        robomas_driver_->setSpeedGainI(i, speed_gain_i_[i]);
      }
      if (i < speed_gain_d_param.size())
      {
        speed_gain_d_[i] = static_cast<float>(speed_gain_d_param[i]);
        robomas_driver_->setSpeedGainD(i, speed_gain_d_[i]);
      }
      if (i < pos_gain_p_param.size())
      {
        pos_gain_p_[i] = static_cast<float>(pos_gain_p_param[i]);
        robomas_driver_->setPosGainP(i, pos_gain_p_[i]);
      }
      if (i < pos_gain_i_param.size())
      {
        pos_gain_i_[i] = static_cast<float>(pos_gain_i_param[i]);
        robomas_driver_->setPosGainI(i, pos_gain_i_[i]);
      }
      if (i < pos_gain_d_param.size())
      {
        pos_gain_d_[i] = static_cast<float>(pos_gain_d_param[i]);
        robomas_driver_->setPosGainD(i, pos_gain_d_[i]);
      }
      if (i < speed_lim_param.size())
      {
        speed_lim_[i] = static_cast<float>(speed_lim_param[i]);
        robomas_driver_->setSpeedLim(i, speed_lim_[i]);
      }
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr sabacan_status_publisher_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr sabacan_ref_subscription_;
  rclcpp::Service<sabacan_msgs::srv::SetRobomasGains>::SharedPtr set_gains_service_;
  rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::shared_ptr<CanDriver> can_driver_;
  std::unique_ptr<RobomasDriverV1> robomas_driver_;

  std::string motor_type[4] = {"Robomas", "Robomas", "Robomas", "Robomas"};
  std::string control_type[4] = {"VELOCITY", "VELOCITY", "VELOCITY", "VELOCITY"};

  // PIDゲインと速度制限の配列
  float speed_gain_p_[4];
  float speed_gain_i_[4];
  float speed_gain_d_[4];
  float pos_gain_p_[4];
  float pos_gain_i_[4];
  float pos_gain_d_[4];
  float speed_lim_[4];
  bool enable_monitor_period_ = true;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SabaneCanNode>());
  rclcpp::shutdown();
  return 0;
}
