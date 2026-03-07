#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan/sabacan.h"
#include "sabacan_msgs/msg/sabacan_gpio_ref_float.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_int.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"

using namespace std::chrono_literals;

class SabacanGPIONode : public rclcpp::Node
{
public:
  SabacanGPIONode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sabacan_gpio_node")
  {
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

    this->declare_parameter("enable_initialize", true);
    this->get_parameter("enable_initialize", enable_initialize_);

    this->declare_parameter("publish_timer_rate", 100.0);
    this->get_parameter("publish_timer_rate", publish_timer_rate_);

    // pin_typeの種類はINPUT, OUTPUT_PWM, OUTPUT_ESC、OUTPUT_SERVOの4つ
    this->declare_parameter(
      "pin_type",
      std::vector<std::string>{
        "INPUT", "INPUT", "INPUT", "INPUT", "INPUT", "INPUT", "INPUT", "INPUT", "INPUT"});

    // pwm_freqのデフォルト値はすべて0Hz（無効）
    this->declare_parameter("pwm_freq", std::vector<int64_t>{0, 0, 0, 0, 0, 0, 0, 0, 0});

    this->declare_parameter("servo_min_angle", std::vector<int64_t>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter(
      "servo_max_angle", std::vector<int64_t>{180, 180, 180, 180, 180, 180, 180, 180, 180});
    this->declare_parameter(
      "servo_min_pulse_width", std::vector<int64_t>{500, 500, 500, 500, 500, 500, 500, 500, 500});
    this->declare_parameter(
      "servo_max_pulse_width",
      std::vector<int64_t>{2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500});

    // デフォルトでは20Hz(50ms)でPORT_READを送信
    this->declare_parameter("monitor_period", 50);
    this->declare_parameter("enable_monitor_period", true);
    int64_t reg = 0LL;
    reg |= 1LL << GPIO::PORT_READ;
    reg |= 1LL << GPIO::PWM_DUTY;
    this->declare_parameter("monitor_reg", reg);

    // map
    pin_type_map_["INPUT"] = PIN_TYPE_INPUT;
    pin_type_map_["OUTPUT_PWM"] = PIN_TYPE_OUTPUT_PWM;
    pin_type_map_["OUTPUT_ESC"] = PIN_TYPE_OUTPUT_ESC;
    pin_type_map_["OUTPUT_SERVO"] = PIN_TYPE_OUTPUT_SERVO;

    // 必須パラメータ'board_id'が設定されているかチェック
    try {
      this->get_parameter("board_id", board_id_);
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      // パラメータが設定されていなかった場合、致命的なエラーを出して終了
      RCLCPP_FATAL(
        this->get_logger(),
        "Mandatory parameter 'board_id' was not set. Please provide it at launch.\n"
        "Example: ros2 run your_package_name sabacan_gpio_node --ros-args -p "
        "board_id:=<value>");
      // コンストラクタで例外を投げることで、ノードの初期化を安全に中止します
      throw;
    }

    can_driver_ = std::make_shared<CanDriver>();
    gpio_driver_ = std::make_shared<GPIODriver>(can_driver_, board_id_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

    // CANデータ受信用のSubscriber
    can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100, std::bind(&SabacanGPIONode::can_callback, this, std::placeholders::_1));

    // 現在のGPIOの状態を送信するPublisher
    sabacan_gpio_status_publisher_ = this->create_publisher<sabacan_msgs::msg::SabacanGPIOStatus>(
      "/sabacan_gpio_status" + std::to_string(board_id_), 100);

    // GPIOの指令値を受け取るSubscriber（float版）
    sabacan_gpio_ref_float_subscription_ =
      this->create_subscription<sabacan_msgs::msg::SabacanGPIORefFloat>(
        "/sabacan_gpio_ref_float" + std::to_string(board_id_), 100,
        std::bind(&SabacanGPIONode::sabacan_gpio_ref_float_callback, this, std::placeholders::_1));

    // GPIOの指令値を受け取るSubscriber（int版）
    sabacan_gpio_ref_int_subscription_ =
      this->create_subscription<sabacan_msgs::msg::SabacanGPIORefInt>(
        "/sabacan_gpio_ref_int" + std::to_string(board_id_), 100,
        std::bind(&SabacanGPIONode::sabacan_gpio_ref_int_callback, this, std::placeholders::_1));

    // リセットサービスサーバーの作成
    reset_service_ = this->create_service<sabacan_msgs::srv::SabacanReset>(
      "sabacan_gpio_reset",
      std::bind(
        &SabacanGPIONode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // パラメータ変更コールバックの設定
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SabacanGPIONode::parameter_callback, this, std::placeholders::_1));

    // publish用のtimerを呼ぶ
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_timer_rate_),
      std::bind(&SabacanGPIONode::publish_timer_callback, this));

    // 初期化命令を送信
    if (enable_initialize_) {
      gpio_init();
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

    uint16_t prev_port_read = gpio_driver_->port_read;

    frame = can_driver_->rx_frame(msg->id, data, msg->dlc, msg->is_rtr, msg->is_extended);
    gpio_driver_->receive(frame);

    // PORT_READが変化したらpublishする
    if (prev_port_read != gpio_driver_->port_read) {
      publish_status_msg();
    }
  }

  void sabacan_gpio_ref_float_callback(const sabacan_msgs::msg::SabacanGPIORefFloat::SharedPtr msg)
  {
    if (msg->pin_number >= N) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Received invalid pin_number %d in SabacanGPIORefFloat message. Ignoring.",
        msg->pin_number);
      return;
    }
    // 指定されたピンがfloat型のrefに対応しているか確認
    if (pin_type_[msg->pin_number] != PIN_TYPE_OUTPUT_PWM) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Pin number %d is not configured for float reference output. Ignoring SabacanGPIORefFloat "
        "message.",
        msg->pin_number);
      return;
    }
    int pin_num = msg->pin_number;
    if (pin_type_[pin_num] == PIN_TYPE_OUTPUT_PWM) {
      // PWM出力の場合
      // duty比が0.0~1.0の範囲であるかを確認
      if (msg->ref_float < 0.0f || msg->ref_float > 1.0f) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Received invalid PWM duty cycle %.3f for pin %d. Must be between 0.0 and 1.0. Ignoring.",
          msg->ref_float, pin_num);
        return;
      }
      // pwm_freqが適切な値であるかをチェック
      if (pwm_freq_[pin_num] <= 0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "PWM frequency for pin %d is not set or invalid (%ld Hz). Cannot calculate PWM period. "
          "Ignoring.",
          pin_num, pwm_freq_[pin_num]);
        return;
      }
      // pwm_periodを計算
      // (1 / pwm_freq_[pin_num]) / (1 / GPIODriver::PWM_COUNTER_FREQ)
      uint16_t pwm_period = GPIODriver::PWM_COUNTER_FREQ / pwm_freq_[pin_num];
      uint16_t pwm_duty = static_cast<uint16_t>(msg->ref_float * pwm_period);

      pwm_duty_[pin_num] = pwm_duty;
      gpio_driver_->setPwmDuty(pin_num, pwm_duty);
      delay();

      RCLCPP_INFO(
        this->get_logger(), "Set PWM duty for pin %d: ref_float=%f, pwm_period=%d, pwm_duty=%d",
        pin_num, msg->ref_float, pwm_period, pwm_duty);
    }
  }

  void sabacan_gpio_ref_int_callback(const sabacan_msgs::msg::SabacanGPIORefInt::SharedPtr msg)
  {
    if (msg->pin_number >= N) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Received invalid pin_number %d in SabacanGPIORefInt message. Ignoring.", msg->pin_number);
      return;
    }
    // 指定されたピンがint型のrefに対応しているか確認
    bool is_esc = (pin_type_[msg->pin_number] == PIN_TYPE_OUTPUT_ESC);
    bool is_servo = (pin_type_[msg->pin_number] == PIN_TYPE_OUTPUT_SERVO);
    if (is_esc == false && is_servo == false) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Pin number %d is not configured for int reference output. Ignoring SabacanGPIORefInt "
        "message.",
        msg->pin_number);
      return;
    }
    int pin_num = msg->pin_number;
    if (pin_type_[pin_num] == PIN_TYPE_OUTPUT_SERVO) {
      // サーボ出力の場合
      int32_t angle = msg->ref_int;
      // サーボは範囲外の角度を入力して微調整したくなるときがあるかもなので、範囲のチェックは行わない
      double min_width = (double)servo_min_pulse_width_[pin_num];
      double max_width = (double)servo_max_pulse_width_[pin_num];
      double min_angle = (double)servo_min_angle_[pin_num];
      double max_angle = (double)servo_max_angle_[pin_num];
      double pulse_width_us =
        min_width + (max_width - min_width) * ((double)angle - min_angle) / (max_angle - min_angle);

      double pulse_width_s = pulse_width_us * 1e-6;
      double duty = pulse_width_s / (1 / (double)pwm_freq_[pin_num]);
      uint16_t pwm_period = GPIODriver::PWM_COUNTER_FREQ / pwm_freq_[pin_num];
      pwm_duty_[pin_num] = duty * (double)pwm_period;
      gpio_driver_->setPwmDuty(pin_num, pwm_duty_[pin_num]);
      RCLCPP_INFO(
        this->get_logger(),
        "Set servo angle for pin %d: ref_int=%d, pulse_width=%.2f us, pwm_period=%d, pwm_duty=%d",
        pin_num, msg->ref_int, pulse_width_us, pwm_period, pwm_duty_[pin_num]);
      delay();

    } else if (pin_type_[pin_num] == PIN_TYPE_OUTPUT_ESC) {
      // ESC出力の場合
      uint16_t esc_value = static_cast<uint16_t>(msg->ref_int);
      // ESCの指令値が50~100の範囲であるかを確認
      if (esc_value < 50 || esc_value > 100) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Received invalid ESC value %d for pin %d. Must be between 50 and 100. Ignoring.",
          esc_value, pin_num);
        return;
      }
      pwm_duty_[pin_num] = esc_value;
      gpio_driver_->setPwmDuty(pin_num, esc_value);
      delay();

      RCLCPP_INFO(
        this->get_logger(), "Set ESC value for pin %d: ref_int=%d, esc_value=%d", pin_num,
        msg->ref_int, esc_value);
    }
  }

  void publish_status_msg()
  {
    // GPIOの状態を取得してメッセージを作成
    auto status_msg = std::make_shared<sabacan_msgs::msg::SabacanGPIOStatus>();
    for (int i = 0; i < N; i++) {
      status_msg->pin_number = (uint8_t)i;
      int input_state = (gpio_driver_->port_read >> i) & 0x01;
      status_msg->input = (bool)input_state;
      // PIN_TYPE_INPUTの場合のみpublish
      if (pin_type_[i] == PIN_TYPE_INPUT) {
        sabacan_gpio_status_publisher_->publish(*status_msg);
      }
    }
  }

  void publish_timer_callback() { publish_status_msg(); }

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
    gpio_init();
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Sabacan GPIO board reset successfully.");
  }

  void gpio_init()
  {
    // clang-format off
    std::vector<std::string> param_name{
      "pin_type",
      "pwm_freq",
      "servo_min_angle",
      "servo_max_angle",
      "servo_min_pulse_width",
      "servo_max_pulse_width",
      "monitor_period",
      "enable_monitor_period",
      "monitor_reg"
    };
    // clang-format on

    rclcpp::Time start_time = this->get_clock()->now();

    for (size_t i = 0; i < param_name.size(); i++) {
      // update_parameters関数内ではdelayが入っている。
      update_parameters(this->get_parameter(param_name[i]));
    }

    rclcpp::Time end_time = this->get_clock()->now();
    RCLCPP_INFO(
      this->get_logger(), "GPIO initialization completed in %.3f seconds",
      (end_time - start_time).seconds());
  }

  void delay() { std::this_thread::sleep_for(10ms); }

  bool update_parameters(rclcpp::Parameter parameter)
  {
    bool ret = true;
    std::string name = parameter.get_name();
    int inf = 1e9;

    if (name == "board_id") {
      RCLCPP_ERROR(this->get_logger(), "Board id can't change.");
      return false;
    } else if (name == "pin_type") {
      auto tmp_param = parameter.as_string_array();
      if (check_size(tmp_param, N, name) == false) return false;
      for (size_t i = 0; i < N; i++) {
        if (pin_type_map_.contains(tmp_param[i]) == false) {
          ret = false;
          RCLCPP_ERROR(
            this->get_logger(), "Invalid pin_type %s at index %ld", tmp_param[i].c_str(), i);
          return false;
        }
      }
      if (ret == false) return false;
      // pin_typeを更新
      for (size_t i = 0; i < N; i++) {
        pin_type_[i] = pin_type_map_[tmp_param[i]];
      }

      // 安全のため、出力をOFFにする。dutyは書き換えない。
      gpio_driver_->setPortWrite(0x000);
      delay();
      uint16_t port_mode = 0x000;
      uint16_t int_en = 0x000;
      uint16_t esc_mode_en = 0x000;
      uint16_t port_write = 0x000;
      for (size_t i = 0; i < N; i++) {
        // CANデバイスに設定を送信
        if (pin_type_[i] == PIN_TYPE_INPUT) {
          // 入力モードを有効にする
          port_mode |= (1 << i);
          // 入力のときはピン割り込みも有効にする
          int_en |= (1 << i);
        } else if (pin_type_[i] == PIN_TYPE_OUTPUT_ESC) {
          // ESCモードを有効にする。
          esc_mode_en |= (1 << i);
        }

        // port_writeを計算
        if (pin_type_[i] != PIN_TYPE_INPUT) {
          port_write |= 1 << i;
        }
      }
      // 出力に設定
      gpio_driver_->setPortMode(port_mode);
      delay();
      gpio_driver_->setPortIntEn(int_en);
      delay();
      gpio_driver_->setEscModeEn(esc_mode_en);
      delay();
      // 出力のdutyを設定、初期値がなぜか0xFFFFとなっているので
      for (size_t i = 0; i < N; i++) {
        gpio_driver_->setPwmDuty(i, pwm_duty_[i]);
        delay();
      }
      // OUTPUT_SERVOに設定したときは、デフォルトで50Hzに設定する
      for (int i = 0; i < N; i++) {
        if (pin_type_[i] == PIN_TYPE_OUTPUT_SERVO) {
          pwm_freq_[i] = 50;
          uint16_t pwm_period = GPIODriver::PWM_COUNTER_FREQ / pwm_freq_[i];
          RCLCPP_INFO(this->get_logger(), "pwm_period = %d", pwm_period);
          gpio_driver_->setPwmPeriod(i, pwm_period);
          delay();
        }
      }
      // 設定が完了したから、出力を有効にする
      gpio_driver_->setPortWrite(port_write);
      delay();

    } else if (name == "pwm_freq") {
      auto tmp_param = parameter.as_integer_array();
      if (check_data_range_and_size<int64_t>(tmp_param, 0, inf, N, name) == false) return false;
      std::vector<uint16_t> pwm_period(N);
      for (size_t i = 0; i < N; i++) {
        pwm_freq_[i] = tmp_param[i];
        if (pin_type_[i] == PIN_TYPE_OUTPUT_PWM) {
          // pwm_freqが0以下でないかを確認
          if (pwm_freq_[i] <= 0) {
            RCLCPP_ERROR(
              this->get_logger(),
              "PWM frequency for pin %ld is set to invalid value (%ld Hz). Must be > 0 Hz.", i,
              pwm_freq_[i]);
            return false;
          }
          // (1 / pwm_freq_[i]) / (1 / GPIODriver::PWM_COUNTER_FREQ);
          pwm_period[i] = GPIODriver::PWM_COUNTER_FREQ / pwm_freq_[i];
        } else if (pin_type_[i] == PIN_TYPE_OUTPUT_SERVO) {
          // サーボの場合、デフォルトで50Hzに設定
          pwm_freq_[i] = 50;
          pwm_period[i] = GPIODriver::PWM_COUNTER_FREQ / pwm_freq_[i];
        } else {
          // それ以外のときは、念の為明示的に0に設定
          pwm_period[i] = 0;
        }
      }
      // pwm_periodをCANデバイスに送信
      for (size_t i = 0; i < N; i++) {
        gpio_driver_->setPwmPeriod(i, pwm_period[i]);
        delay();
      }
    } else if (name == "servo_min_angle") {
      auto tmp_param = parameter.as_integer_array();
      if (check_data_range_and_size<int64_t>(tmp_param, 0, 360, N, name) == false) return false;
      servo_min_angle_ = tmp_param;
    } else if (name == "servo_max_angle") {
      auto tmp_param = parameter.as_integer_array();
      if (check_data_range_and_size<int64_t>(tmp_param, 0, 360, N, name) == false) return false;
      servo_max_angle_ = tmp_param;
    } else if (name == "servo_min_pulse_width") {
      auto tmp_param = parameter.as_integer_array();
      if (check_data_range_and_size<int64_t>(tmp_param, 0, inf, N, name) == false) return false;
      servo_min_pulse_width_ = tmp_param;
    } else if (name == "servo_max_pulse_width") {
      auto tmp_param = parameter.as_integer_array();
      if (check_data_range_and_size<int64_t>(tmp_param, 0, inf, N, name) == false) return false;
      servo_max_pulse_width_ = tmp_param;
    } else if (name == "monitor_period") {
      int64_t tmp_param = parameter.as_int();
      if (check_data_range<int64_t>(tmp_param, 0, inf, name) == false) return false;
      monitor_period_ = tmp_param;
      gpio_driver_->setMonitorPeriod(static_cast<uint16_t>(monitor_period_));
      delay();
    } else if (name == "monitor_reg") {
      int64_t tmp_param = parameter.as_int();
      monitor_reg_ = tmp_param;
      gpio_driver_->setMonitorReg(static_cast<uint64_t>(monitor_reg_));
      delay();
    } else if (name == "enable_monitor_period") {
      enable_monitor_period_ = parameter.as_bool();
      if (enable_monitor_period_ == false) {
        gpio_driver_->setMonitorPeriod(0);  // 無効化
        delay();
      } else {
        gpio_driver_->setMonitorPeriod(static_cast<uint16_t>(monitor_period_));
        delay();
        gpio_driver_->setMonitorReg(static_cast<uint64_t>(monitor_reg_));
        delay();
      }
    }
    return ret;
  }

private:
  static constexpr int PIN_TYPE_INPUT = 0;
  static constexpr int PIN_TYPE_OUTPUT_PWM = 1;
  static constexpr int PIN_TYPE_OUTPUT_ESC = 2;
  static constexpr int PIN_TYPE_OUTPUT_SERVO = 3;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr sabacan_gpio_status_publisher_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIORefInt>::SharedPtr
    sabacan_gpio_ref_int_subscription_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIORefFloat>::SharedPtr
    sabacan_gpio_ref_float_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<CanDriver> can_driver_;
  std::shared_ptr<GPIODriver> gpio_driver_;
  rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  static constexpr int N = 9;
  int64_t board_id_;
  bool enable_initialize_;
  double publish_timer_rate_;
  // map
  std::map<std::string, int> pin_type_map_;
  // パラメータ、サービスで使用する変数
  std::vector<int64_t> pin_type_ = std::vector<int64_t>(N);
  std::vector<int64_t> pwm_freq_ = std::vector<int64_t>(N);
  std::vector<int64_t> servo_min_angle_ = std::vector<int64_t>(N);
  std::vector<int64_t> servo_max_angle_ = std::vector<int64_t>(N);
  std::vector<int64_t> servo_min_pulse_width_ = std::vector<int64_t>(N);
  std::vector<int64_t> servo_max_pulse_width_ = std::vector<int64_t>(N);
  bool enable_monitor_period_;
  int64_t monitor_period_;
  int64_t monitor_reg_;
  std::vector<uint16_t> pwm_duty_ = std::vector<uint16_t>(N);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();

  // options をコンストラクタに渡す
  rclcpp::spin(std::make_shared<SabacanGPIONode>(options));

  rclcpp::shutdown();
  return 0;
}
