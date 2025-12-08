#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan/sabacan.h"
#include "sabacan_msgs/msg/sabacan_robomas_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_msgs/srv/set_robomas_gains.hpp"

using namespace std::chrono_literals;

class SabacanRobomasV2Node : public rclcpp::Node
{
public:
  explicit SabacanRobomasV2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sabacan_robomasv2_node", options)  // options を基底クラスに渡す
  {
    // パラメータの宣言

    // board_idを必須パラメータとして宣言（デフォルト値なし）
    // パラメータの制約や説明を記述子で定義
    auto board_id_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    board_id_descriptor.description =
      "The unique ID of the CAN board (0-9). This parameter is mandatory.";
    board_id_descriptor.integer_range.resize(1);
    board_id_descriptor.integer_range[0].from_value = 0;
    board_id_descriptor.integer_range[0].to_value = 9;
    board_id_descriptor.integer_range[0].step = 1;
    this->declare_parameter<int64_t>("board_id", board_id_descriptor);

    this->declare_parameter("motor_type", std::vector<std::string>{"C610", "C610", "C610", "C610"});
    this->declare_parameter(
      "control_type", std::vector<std::string>{"VELOCITY", "VELOCITY", "VELOCITY", "VELOCITY"});
    this->declare_parameter("dob_en", std::vector<bool>{false, false, false, false});
    this->declare_parameter("abs_enc_en", std::vector<bool>{false, false, false, false});
    this->declare_parameter("md_guess_en", std::vector<bool>{false, false, false, false});
    this->declare_parameter("abs_gear_ratio", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    this->declare_parameter("cal_rq", std::vector<bool>{false, false, false, false});
    this->declare_parameter("load_j", std::vector<double>{0.0005, 0.0005, 0.0005, 0.0005});
    this->declare_parameter("load_d", std::vector<double>{0.0004, 0.0004, 0.0004, 0.0004});
    this->declare_parameter("dob_cf", std::vector<double>{5.0, 5.0, 5.0, 5.0});
    // PIDゲインパラメータの宣言
    this->declare_parameter("speed_gain_p", std::vector<double>{0.5, 0.5, 0.5, 0.5});
    this->declare_parameter("speed_gain_i", std::vector<double>{0.2, 0.2, 0.2, 0.2});
    this->declare_parameter("speed_gain_d", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("torque_lim", std::vector<double>{5.0, 5.0, 5.0, 5.0});
    this->declare_parameter("pos_gain_p", std::vector<double>{6.0, 6.0, 6.0, 6.0});
    this->declare_parameter("pos_gain_i", std::vector<double>{3.0, 3.0, 3.0, 3.0});
    this->declare_parameter("pos_gain_d", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("speed_lim", std::vector<double>{30.0, 30.0, 30.0, 30.0});
    this->declare_parameter("abs_turn_cnt", std::vector<int64_t>{0, 0, 0, 0});
    this->declare_parameter("vesc_pole", std::vector<int64_t>{14, 14, 14, 14});
    // モニター機能のパラメータ
    // デフォルトでは20Hz(50ms)で、MOTOR_STATE、TRQ、SPD、POS、ABS_POS、ABS_SPD、ABS_TURN_CNT、VESC_VOLTAGE、VESC_CURRENT、VESC_ERPMを送信
    this->declare_parameter("monitor_period", 50);
    // モニタ周期設定の有効/無効を制御するパラメータ
    this->declare_parameter("enable_monitor_period", true);  // デフォルトtrue
    int64_t reg1 = 0LL;
    reg1 |= 1LL << RobomasV2::MOTOR_STATE;
    reg1 |= 1LL << RobomasV2::TRQ;
    reg1 |= 1LL << RobomasV2::SPD;
    reg1 |= 1LL << RobomasV2::POS;
    reg1 |= 1LL << RobomasV2::ABS_POS;
    reg1 |= 1LL << RobomasV2::ABS_SPD;
    reg1 |= 1LL << RobomasV2::ABS_TURN_CNT;
    this->declare_parameter("monitor_reg1", std::vector<int64_t>{reg1, reg1, reg1, reg1});
    int64_t reg2 = 0LL;
    reg2 |= 1LL << (RobomasV2::VESC_VOLTAGE - 0x40);
    reg2 |= 1LL << (RobomasV2::VESC_CURRENT - 0x40);
    reg2 |= 1LL << (RobomasV2::VESC_ERPM - 0x40);
    this->declare_parameter("monitor_reg2", std::vector<int64_t>{reg2, reg2, reg2, reg2});
    // map
    motor_type_map_["C610"] = RobomasV2::CONTROL_MOTOR_C610;
    motor_type_map_["C620"] = RobomasV2::CONTROL_MOTOR_C620;
    motor_type_map_["VESC"] = RobomasV2::CONTROL_MOTOR_VESC;
    control_type_map_["TORQUE"] = RobomasV2::CONTROL_MODE_TRQ;
    // VELOCITYとSPEEDはプロトコル上はSPEED(SPD)が正しいが、以前のコードと互換性を保つために、VELOCITYも残す
    control_type_map_["VELOCITY"] = RobomasV2::CONTROL_MODE_SPD;
    control_type_map_["SPEED"] = RobomasV2::CONTROL_MODE_SPD;
    control_type_map_["POS"] = RobomasV2::CONTROL_MODE_POS;
    control_type_map_["POSITION"] = RobomasV2::CONTROL_MODE_POS;
    vesc_mode_map_["DISABLE"] = RobomasV2::VESC_MODE_DISABLE;
    vesc_mode_map_["PWM"] = RobomasV2::VESC_MODE_PWM;
    vesc_mode_map_["CURRENT"] = RobomasV2::VESC_MODE_CURRENT;
    // VELOCITYとSPEEDはプロトコル上はSPEED(SPD)が正しいが、以前のコードと互換性を保つために、VELOCITYも残す
    vesc_mode_map_["SPEED"] = RobomasV2::VESC_MODE_SPEED;
    vesc_mode_map_["VELOCITY"] = RobomasV2::VESC_MODE_SPEED;
    vesc_mode_map_["POS"] = RobomasV2::VESC_MODE_POSITION;
    vesc_mode_map_["POSITION"] = RobomasV2::VESC_MODE_POSITION;

    // 必須パラメータ'board_id'が設定されているかチェック
    try {
      this->get_parameter("board_id", board_id_);
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      // パラメータが設定されていなかった場合、致命的なエラーを出して終了
      RCLCPP_FATAL(
        this->get_logger(),
        "Mandatory parameter 'board_id' was not set. Please provide it at launch.\n"
        "Example: ros2 run your_package_name sabacan_robomasv2_node --ros-args -p "
        "board_id:=<value>");
      // コンストラクタで例外を投げることで、ノードの初期化を安全に中止します
      throw;
    }

    can_driver_ = std::make_shared<CanDriver>();
    robomas_driver_ = std::make_unique<RobomasDriverV2>(can_driver_, board_id_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

    // CANデータ受信用のSubscriber
    can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100,
      std::bind(&SabacanRobomasV2Node::can_callback, this, std::placeholders::_1));

    // 現在のモータの状態を送信するPublisher
    sabacan_status_publisher_ = this->create_publisher<sabacan_msgs::msg::SabacanRobomasStatus>(
      "/sabacan_robomas_status" + std::to_string(board_id_), 100);

    // モータの指令値を受け取るSubscriber
    sabacan_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanRobomasRef>(
      "/sabacan_robomas_ref" + std::to_string(board_id_), 100,
      std::bind(&SabacanRobomasV2Node::sabacan_ref_callback, this, std::placeholders::_1));

    // サービスサーバーの作成
    set_gains_service_ = this->create_service<sabacan_msgs::srv::SetRobomasGains>(
      "set_robomas_gains", std::bind(
                             &SabacanRobomasV2Node::set_gains_callback, this, std::placeholders::_1,
                             std::placeholders::_2));

    // リセットサービスサーバーの作成
    reset_service_ = this->create_service<sabacan_msgs::srv::SabacanReset>(
      "sabacan_robomas_reset",
      std::bind(
        &SabacanRobomasV2Node::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // パラメータ変更コールバックの設定
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SabacanRobomasV2Node::parameter_callback, this, std::placeholders::_1));

    // 100Hzでpublish用のtimerを呼ぶ
    publish_timer_ =
      this->create_wall_timer(10ms, std::bind(&SabacanRobomasV2Node::publish_timer_callback, this));

    // 初期化命令を送信
    robomas_init();
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

private:
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
    CanFrame frame;
    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
      data[i] = msg->data[i];
    }
    frame = can_driver_->rx_frame(msg->id, data, msg->dlc, msg->is_rtr, msg->is_extended);
    robomas_driver_->receive(frame);
  }

  /**
   * @brief トピックで目標値が更新されたときのcallback
   *
   * @param msg
   */
  void sabacan_ref_callback(const sabacan_msgs::msg::SabacanRobomasRef::SharedPtr msg)
  {
    if (check_data_range<int>(msg->motor_number, 0, N, "motor_number") == false) return;
    bool is_dji_motor =
      (motor_type_[msg->motor_number] == RobomasV2::CONTROL_MOTOR_C610 ||
       motor_type_[msg->motor_number] == RobomasV2::CONTROL_MOTOR_C620);
    bool is_vesc = (motor_type_[msg->motor_number] == RobomasV2::CONTROL_MOTOR_VESC);
    if (is_dji_motor) {
      switch (control_type_[msg->motor_number]) {
        case RobomasV2::CONTROL_MODE_SPD:
          robomas_driver_->setSpeedTarget(msg->motor_number, msg->ref);
          break;
        case RobomasV2::CONTROL_MODE_POS:
          robomas_driver_->setPosTarget(msg->motor_number, msg->ref);
          break;
        case RobomasV2::CONTROL_MODE_TRQ:
          robomas_driver_->setTorqueTarget(msg->motor_number, msg->ref);
          break;
      }
    } else if (is_vesc) {
      if (vesc_mode_[msg->motor_number] == RobomasV2::VESC_MODE_SPEED) {
        // rad/sからrpmに変換
        double rpm = msg->ref * (60.0 / (2.0 * M_PI));
        // rpmからVESCの指令値であるerpmに変換
        // erpm = rpm * (p / 2)
        double erpm = rpm * (vesc_pole_[msg->motor_number] / 2.0);
        robomas_driver_->setVescTarget(msg->motor_number, erpm);  // rpm to erpm
      } else
        robomas_driver_->setVescTarget(msg->motor_number, msg->ref);
    }

    RCLCPP_INFO(
      this->get_logger(), "Received SabacanRef: motor_number=%d, ref=%f", msg->motor_number,
      msg->ref);
  }

  void publish_timer_callback(void)
  {
    for (int i = 0; i < N; i++) {
      auto msg = sabacan_msgs::msg::SabacanRobomasStatus();
      msg.motor_number = i;
      msg.motor_type = motor_type_name_[i];
      msg.control_type = control_type_name_[i];
      msg.motor_state = robomas_driver_->motor_state[i];
      msg.torque = robomas_driver_->current_torque[i];
      msg.speed = robomas_driver_->current_speed[i];
      msg.pos = robomas_driver_->current_pos[i];
      msg.abs_pos = robomas_driver_->abs_pos[i];
      msg.abs_speed = robomas_driver_->abs_speed[i];
      msg.abs_turn_cnt = robomas_driver_->abs_turn_cnt[i];
      msg.vesc_voltage = robomas_driver_->vesc_voltage[i];
      msg.vesc_current = robomas_driver_->vesc_current[i];
      // VESCの生データerpmからrpmに変換
      // rpm = erpm / (p / 2)
      double rpm = robomas_driver_->vesc_erpm[i] / (vesc_pole_[i] / 2.0);
      // rpmからrad/sに変換
      double rad_s = rpm * (2.0 * M_PI / 60.0);
      msg.vesc_speed = rad_s;
      sabacan_status_publisher_->publish(msg);
      // RCLCPP_INFO(this->get_logger(), "Published SabacanRobomasStatus%ld, motor_number = %d",
      // board_id_, i);
    }
  }

  /**
   * @brief 遅延処理。送信と送信の間などに使用する
   * 
   */
  void delay() { std::this_thread::sleep_for(10ms); }

  /**
   * @brief 初期化命令を基板に送信する
   *
   */
  void robomas_init()
  {
    // 初期化用パラメータ
    // clang-format off
    std::vector<std::string> param_name{
      "motor_type",
      "control_type",
      "dob_en",
      "abs_enc_en",
      "md_guess_en",
      "abs_gear_ratio",
      "cal_rq",
      "load_j",
      "load_d",
      "dob_cf",
      "speed_gain_p",
      "speed_gain_i",
      "speed_gain_d",
      "torque_lim",
      "pos_gain_p",
      "pos_gain_i",
      "pos_gain_d",
      "speed_lim",
      "abs_turn_cnt",
      "vesc_pole",
      "monitor_period",
      "monitor_reg1",
      "monitor_reg2",
      "enable_monitor_period"
    };
    // clang-format on

    rclcpp::Time start_time = this->get_clock()->now();

    for (size_t i = 0; i < param_name.size(); i++) {
      // update_parametersの中では、CANのデータを1つ送信するごとにdelayを読んでいる。
      // 負荷軽減のため。
      update_parameters(this->get_parameter(param_name[i]));
    }

    rclcpp::Time end_time = this->get_clock()->now();
    RCLCPP_INFO(
      this->get_logger(), "RobomasV2 initialization completed in %.3f seconds",
      (end_time - start_time).seconds());
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

  // サービスコールバック
  void set_gains_callback(
    const std::shared_ptr<sabacan_msgs::srv::SetRobomasGains::Request> request,
    std::shared_ptr<sabacan_msgs::srv::SetRobomasGains::Response> response)
  {
    if (request->motor_number >= N) {
      response->success = false;
      response->message = "Invalid motor number. Must be 0-" + std::to_string(N - 1) + ".";
      return;
    }

    try {
      int n = request->motor_number;

      if (request->set_dob_param) {
        if (request->dob_load_j < 0.0) {
          response->message = "Invalid dob_load_j";
          response->success = false;
          return;
        }
        if (request->dob_load_d < 0.0) {
          response->message = "Invalid dob_load_d";
          response->success = false;
          return;
        }
        if (request->dob_cutoff_freq < 0.0) {
          response->message = "Invalid dob_cutoff_freq";
          response->success = false;
          return;
        }
        load_j_[n] = request->dob_load_j;
        load_d_[n] = request->dob_load_d;
        dob_cf_[n] = request->dob_cutoff_freq;
        robomas_driver_->setLoad_J(n, load_j_[n]);
        delay();
        robomas_driver_->setLoad_D(n, load_d_[n]);
        delay();
        robomas_driver_->setDob_CF(n, dob_cf_[n]);
        delay();
        this->set_parameter(rclcpp::Parameter("load_j", load_j_));
        this->set_parameter(rclcpp::Parameter("load_d", load_d_));
        this->set_parameter(rclcpp::Parameter("dob_cf", dob_cf_));
      }

      if (request->set_speed_gains) {
        if (request->torque_lim < 0.0) {
          response->message = "Invalid torque lim";
          response->success = false;
          return;
        }
        speed_gain_p_[n] = request->speed_gain_p;
        speed_gain_i_[n] = request->speed_gain_i;
        speed_gain_d_[n] = request->speed_gain_d;
        torque_lim_[n] = request->torque_lim;
        robomas_driver_->setSpeedGainP(n, speed_gain_p_[n]);
        delay();
        robomas_driver_->setSpeedGainI(n, speed_gain_i_[n]);
        delay();
        robomas_driver_->setSpeedGainD(n, speed_gain_d_[n]);
        delay();
        robomas_driver_->setTorqueLimit(n, torque_lim_[n]);
        delay();
        this->set_parameter(rclcpp::Parameter("speed_gain_p", speed_gain_p_));
        this->set_parameter(rclcpp::Parameter("speed_gain_i", speed_gain_i_));
        this->set_parameter(rclcpp::Parameter("speed_gain_d", speed_gain_d_));
        this->set_parameter(rclcpp::Parameter("torque_lim", torque_lim_));
      }

      if (request->set_pos_gains) {
        if (request->speed_lim < 0.0) {
          response->message = "Invalid speed lim";
          response->success = false;
          return;
        }
        pos_gain_p_[n] = request->pos_gain_p;
        pos_gain_i_[n] = request->pos_gain_i;
        pos_gain_d_[n] = request->pos_gain_d;
        speed_lim_[n] = request->speed_lim;

        robomas_driver_->setPosGainP(n, pos_gain_p_[n]);
        delay();
        robomas_driver_->setPosGainI(n, pos_gain_i_[n]);
        delay();
        robomas_driver_->setPosGainD(n, pos_gain_d_[n]);
        delay();
        robomas_driver_->setSpeedLim(n, speed_lim_[n]);
        delay();
        this->set_parameter(rclcpp::Parameter("speed_lim", speed_lim_));
      }

      if (request->set_abs_turn_cnt) {
        abs_turn_cnt_[n] = request->abs_turn_cnt;
        robomas_driver_->setAbsTurnCnt(n, abs_turn_cnt_[n]);
        delay();
      }

      response->success = true;
      response->message = "Gains updated successfully for motor " + std::to_string(n);

      RCLCPP_INFO(this->get_logger(), "Updated gains for motor %d via service", n);
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("Error updating gains: ") + e.what();
    }
  }

  // リセットサービスコールバック
  void reset_callback(
    const std::shared_ptr<sabacan_msgs::srv::SabacanReset::Request> request,
    std::shared_ptr<sabacan_msgs::srv::SabacanReset::Response> response)
  {
    (void)request;  // 未使用パラメータ警告を抑制
    try {
      RCLCPP_INFO(this->get_logger(), "Received reset request for Robomas node");

      // Robomas初期化処理を実行
      robomas_init();

      response->success = true;
      response->message = "Robomas node reset completed successfully";

      RCLCPP_INFO(this->get_logger(), "Robomas node reset completed");
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("Error during Robomas reset: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Robomas reset failed: %s", e.what());
    }
  }

  /**
   * @brief パラメータを更新する。
   * 値の代入と値が正しいかを判別するロジックが混ざっているが、分けると実装料が増えて逆に管理が大変になると考えたため。
   *
   * @param parameter
   * @return true
   * @return false
   */
  bool update_parameters(rclcpp::Parameter parameter)
  {
    double inf = 1e18;
    bool ret = true;
    std::string name = parameter.get_name();

    if (name == "board_id") {
      RCLCPP_ERROR(this->get_logger(), "Board id can't change.");
      return false;
    } else if (name == "motor_type") {
      auto tmp_param = parameter.as_string_array();
      // パラメータが正しいかを確認
      if (check_size(tmp_param, N, name) == false) return false;
      for (size_t i = 0; i < N; i++) {
        if (motor_type_map_.contains(tmp_param[i]) == false) {
          ret = false;
          RCLCPP_ERROR(
            this->get_logger(), "motor type parameter[%ld] failed: %s", i, tmp_param[i].c_str());
        }
      }
      if (ret == false) return false;
      // motor_typeを更新
      for (size_t i = 0; i < N; i++)
        motor_type_[i] = motor_type_map_[motor_type_name_[i] = tmp_param[i]];

      for (int i = 0; i < N; i++) {
        // motor_typeがロボマスのときのみ送信
        bool is_dji_motor =
          (motor_type_[i] == RobomasV2::CONTROL_MOTOR_C610 ||
           motor_type_[i] == RobomasV2::CONTROL_MOTOR_C620);
        if (is_dji_motor) {
          robomas_driver_->setControl(
            i, (uint8_t)control_type_[i], (uint8_t)motor_type_[i], dob_en_[i], abs_enc_en_[i],
            md_guess_en_[i]);
          delay();
        }
      }

    } else if (name == "control_type") {
      auto tmp_param = parameter.as_string_array();
      // パラメータが正しいか確認
      if (check_size(control_type_, N, "control_type") == false) return false;
      std::vector<bool> is_dji_motor(N);
      std::vector<bool> is_vesc(N);
      std::vector<bool> is_dji_motor_map_ok(N);
      std::vector<bool> is_vesc_map_ok(N);
      for (size_t i = 0; i < N; i++) {
        // モータの種類を判定
        is_dji_motor[i] =
          (motor_type_[i] == RobomasV2::CONTROL_MOTOR_C610 ||
           motor_type_[i] == RobomasV2::CONTROL_MOTOR_C620);
        is_vesc[i] = (motor_type_[i] == RobomasV2::CONTROL_MOTOR_VESC);
        // 設定したcontrol_typeがモータの種類に対応しているかを確認
        is_dji_motor_map_ok[i] = (is_dji_motor[i] && control_type_map_.contains(tmp_param[i]));
        is_vesc_map_ok[i] = (is_vesc[i] && vesc_mode_map_.contains(tmp_param[i]));
        // 対応していないcontrol_typeの場合はエラー
        if (is_dji_motor_map_ok[i] == false && is_vesc_map_ok[i] == false) {
          ret = false;
          RCLCPP_ERROR(
            this->get_logger(), "control type parameter[%ld] failed: %s", i, tmp_param[i].c_str());
        }
      }
      if (ret == false) return ret;

      for (size_t i = 0; i < N; i++) {
        // 変数を更新
        control_type_name_[i] = tmp_param[i];
        if (is_dji_motor_map_ok[i]) {
          // control_typeを更新、vesc_modeはVESC_MODE_DISABLEに設定
          control_type_[i] = control_type_map_[control_type_name_[i]];
          vesc_mode_[i] = RobomasV2::VESC_MODE_DISABLE;
        } else if (is_vesc_map_ok[i]) {
          // control_typeは設定せず、vesc_modeのみ設定
          vesc_mode_[i] = vesc_mode_map_[control_type_name_[i]];
        }
        // 設定を送信
        // control_typeが切り替わったときは、急いでいる可能性があるので、delayは入れない
        robomas_driver_->setControl(
          i, (uint8_t)control_type_[i], (uint8_t)motor_type_[i], dob_en_[i], abs_enc_en_[i],
          md_guess_en_[i]);
        robomas_driver_->setVescMode(i, vesc_mode_[i]);
      }
    } else if (name == "dob_en") {
      auto tmp_param = parameter.as_bool_array();
      if ((ret = check_size(tmp_param, N, name))) {
        dob_en_ = tmp_param;
        for (int i = 0; i < N; i++) {
          robomas_driver_->setControl(
            i, (uint8_t)control_type_[i], (uint8_t)motor_type_[i], dob_en_[i], abs_enc_en_[i],
            md_guess_en_[i]);
          delay();
        }
      }
    } else if (name == "abs_enc_en") {
      auto tmp_param = parameter.as_bool_array();
      if ((ret = check_size(tmp_param, N, name))) {
        abs_enc_en_ = tmp_param;
        for (int i = 0; i < N; i++) {
          robomas_driver_->setControl(
            i, (uint8_t)control_type_[i], (uint8_t)motor_type_[i], dob_en_[i], abs_enc_en_[i],
            md_guess_en_[i]);
          delay();
        }
      }
    } else if (name == "md_guess_en") {
      auto tmp_param = parameter.as_bool_array();
      if ((ret = check_size(tmp_param, N, name))) {
        md_guess_en_ = tmp_param;
        for (int i = 0; i < N; i++) {
          robomas_driver_->setControl(
            i, (uint8_t)control_type_[i], (uint8_t)motor_type_[i], dob_en_[i], abs_enc_en_[i],
            md_guess_en_[i]);
          delay();
        }
      }
    } else if (name == "abs_gear_ratio") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_data_range_and_size(tmp_param, 0.0, inf, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setAbsGearRatio(i, abs_gear_ratio_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "cal_rq") {
      auto tmp_param = parameter.as_bool_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setCalRq(i, cal_rq_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "load_j") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_data_range_and_size(tmp_param, 0.0, inf, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setLoad_J(i, load_j_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "load_d") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_data_range_and_size(tmp_param, 0.0, inf, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setLoad_D(i, load_d_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "dob_cf") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_data_range_and_size(dob_cf_, 0.0, inf, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setDob_CF(i, dob_cf_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "speed_gain_p") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setSpeedGainP(i, speed_gain_p_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "speed_gain_i") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setSpeedGainI(i, speed_gain_i_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "speed_gain_d") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setSpeedGainD(i, speed_gain_d_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "torque_lim") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_data_range_and_size(tmp_param, 0.0, inf, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setTorqueLimit(i, torque_lim_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "pos_gain_p") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setPosGainP(i, pos_gain_p_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "pos_gain_i") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setPosGainI(i, pos_gain_i_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "pos_gain_d") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setPosGainD(i, pos_gain_d_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "speed_lim") {
      auto tmp_param = parameter.as_double_array();
      if ((ret = check_data_range_and_size(tmp_param, 0.0, inf, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setSpeedLim(i, speed_lim_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "abs_turn_cnt") {
      auto tmp_param = parameter.as_integer_array();
      if ((ret = check_size(tmp_param, N, name))) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setAbsTurnCnt(i, abs_turn_cnt_[i] = tmp_param[i]);
          delay();
        }
      }
    } else if (name == "vesc_pole") {
      auto tmp_param = parameter.as_integer_array();
      if ((ret = check_size(tmp_param, N, name))) vesc_pole_ = tmp_param;
    } else if (name == "monitor_period") {
      auto tmp_param = parameter.as_int();
      if ((ret = check_data_range<int64_t>(tmp_param, 0, 65536, name))) {
        monitor_period_ = tmp_param;
        if (enable_monitor_period_) {
          robomas_driver_->setMonitorPeriod(monitor_period_);
          delay();
        }
      }
    } else if (name == "monitor_reg1") {
      auto tmp_param = parameter.as_integer_array();
      monitor_reg1_ = tmp_param;
      if (enable_monitor_period_) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setMonitorReg1(i, monitor_reg1_[i]);
          delay();
        }
      }
    } else if (name == "monitor_reg2") {
      auto tmp_param = parameter.as_integer_array();
      monitor_reg2_ = tmp_param;
      if (enable_monitor_period_) {
        for (int i = 0; i < N; i++) {
          robomas_driver_->setMonitorReg2(i, monitor_reg2_[i]);
          delay();
        }
      }
    } else if (name == "enable_monitor_period") {
      auto tmp_param = parameter.as_bool();
      // enable_monitor_periodがtrueの場合のみ、monitor_periodとmonitor_regを設定
      if ((enable_monitor_period_ = tmp_param)) {
        robomas_driver_->setMonitorPeriod(monitor_period_);
        delay();
        for (int i = 0; i < N; i++) {
          robomas_driver_->setMonitorReg1(i, monitor_reg1_[i]);
          delay();
          robomas_driver_->setMonitorReg2(i, monitor_reg2_[i]);
          delay();
        }
      }
    } else {
      ret = false;
    }
    return ret;
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr sabacan_ref_subscription_;
  rclcpp::Service<sabacan_msgs::srv::SetRobomasGains>::SharedPtr set_gains_service_;
  rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr sabacan_status_publisher_;
  std::shared_ptr<CanDriver> can_driver_;
  std::unique_ptr<RobomasDriverV2> robomas_driver_;

  static constexpr int N = 4;

  // map
  std::map<std::string, int> motor_type_map_;
  std::map<std::string, int> control_type_map_;
  std::map<std::string, int> vesc_mode_map_;
  // 各種データの配列
  int64_t board_id_;
  // パラメータ、サービスで使用する変数
  std::vector<int64_t> motor_type_ = std::vector<int64_t>(N);
  std::vector<int64_t> control_type_ = std::vector<int64_t>(N);
  std::vector<std::string> motor_type_name_ = std::vector<std::string>(N);
  std::vector<std::string> control_type_name_ = std::vector<std::string>(N);
  std::vector<bool> dob_en_ = std::vector<bool>(N);
  std::vector<bool> abs_enc_en_ = std::vector<bool>(N);
  std::vector<bool> md_guess_en_ = std::vector<bool>(N);
  std::vector<double> abs_gear_ratio_ = std::vector<double>(N);
  std::vector<bool> cal_rq_ = std::vector<bool>(N);
  std::vector<double> load_j_ = std::vector<double>(N);
  std::vector<double> load_d_ = std::vector<double>(N);
  std::vector<double> dob_cf_ = std::vector<double>(N);
  std::vector<double> speed_gain_p_ = std::vector<double>(N);
  std::vector<double> speed_gain_i_ = std::vector<double>(N);
  std::vector<double> speed_gain_d_ = std::vector<double>(N);
  std::vector<double> torque_lim_ = std::vector<double>(N);
  std::vector<double> pos_gain_p_ = std::vector<double>(N);
  std::vector<double> pos_gain_i_ = std::vector<double>(N);
  std::vector<double> pos_gain_d_ = std::vector<double>(N);
  std::vector<double> speed_lim_ = std::vector<double>(N);
  std::vector<int64_t> abs_turn_cnt_ = std::vector<int64_t>(N);
  std::vector<int64_t> vesc_pole_ = std::vector<int64_t>(N);
  std::vector<int64_t> vesc_mode_ = std::vector<int64_t>(N);
  int64_t monitor_period_;
  std::vector<int64_t> monitor_reg1_ = std::vector<int64_t>(N);
  std::vector<int64_t> monitor_reg2_ = std::vector<int64_t>(N);
  bool enable_monitor_period_ = true;
};
