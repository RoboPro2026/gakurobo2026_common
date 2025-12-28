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
#include "sabacan_msgs/msg/sabacan_led_mode.hpp"
#include "sabacan_msgs/msg/sabacan_led_ref.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"

using namespace std::chrono_literals;

class SabacanLEDNode : public rclcpp::Node
{
public:
  SabacanLEDNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sabacan_led_node")
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

    this->declare_parameter<bool>("enable_auto_transition", true);
    this->declare_parameter<int64_t>("emg_blink_period", 0);
    // emg_color
    // 面倒なので、emg_colorはすべてのLEDで統一する
    this->declare_parameter("emg_color", std::vector<int64_t>{0x40, 0x00, 0x00});

    // デフォルトでは何もmonitor_regは設定しない
    this->declare_parameter("monitor_period", 50);
    this->declare_parameter("enable_monitor_period", true);
    int64_t reg1 = 0LL;
    this->declare_parameter("monitor_reg1", reg1);
    int64_t reg2 = 0LL;
    this->declare_parameter("monitor_reg2", reg2);

    // 必須パラメータ'board_id'が設定されているかチェック
    try {
      this->get_parameter("board_id", board_id_);
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      // パラメータが設定されていなかった場合、致命的なエラーを出して終了
      RCLCPP_FATAL(
        this->get_logger(),
        "Mandatory parameter 'board_id' was not set. Please provide it at launch.\n"
        "Example: ros2 run your_package_name sabacan_led_node --ros-args -p "
        "board_id:=<value>");
      // コンストラクタで例外を投げることで、ノードの初期化を安全に中止します
      throw;
    }

    can_driver_ = std::make_shared<CanDriver>();
    led_driver_ = std::make_shared<LEDDriver>(can_driver_, board_id_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

    // CANデータ受信用のSubscriber
    can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100, std::bind(&SabacanLEDNode::can_callback, this, std::placeholders::_1));

    // LED modeを取得するSubscriber
    sabacan_led_mode_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanLEDMode>(
      "/sabacan_led_mode" + std::to_string(board_id_), 10,
      std::bind(&SabacanLEDNode::sabacan_led_mode_callback, this, std::placeholders::_1));

    // LEDの指令値を取得するSubscriber
    sabacan_led_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanLEDRef>(
      "/sabacan_led_ref" + std::to_string(board_id_), 10,
      std::bind(&SabacanLEDNode::sabacan_led_ref_callback, this, std::placeholders::_1));

    // リセットサービスサーバーの作成
    reset_service_ = this->create_service<sabacan_msgs::srv::SabacanReset>(
      "sabacan_led_reset" + std::to_string(board_id_),
      std::bind(
        &SabacanLEDNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // パラメータ変更コールバックの設定
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SabacanLEDNode::parameter_callback, this, std::placeholders::_1));

    // 初期化命令を送信
    led_init();
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

    frame = can_driver_->rx_frame(msg->id, data, msg->dlc, msg->is_rtr, msg->is_extended);
    led_driver_->receive(frame);
  }

  void sabacan_led_mode_callback(const sabacan_msgs::msg::SabacanLEDMode::SharedPtr msg)
  {
    led_driver_->setLedMode(msg->led_mode);
    RCLCPP_INFO(this->get_logger(), "Set LED Mode: led_mode=%d", msg->led_mode);
  }

  void sabacan_led_ref_callback(const sabacan_msgs::msg::SabacanLEDRef::SharedPtr msg)
  {
    if (msg->pin_number >= M) {
      RCLCPP_ERROR(this->get_logger(), "pin_number is out of range. value = %d", msg->pin_number);
      return;
    }
    led_driver_->setLedRef(msg->pin_number, msg->start, msg->length, msg->r, msg->g, msg->b);
    RCLCPP_INFO(
      this->get_logger(), "Set LED Ref: pin_number=%d, start=%d, length=%d, r=%d, g=%d, b=%d",
      msg->pin_number, msg->start, msg->length, msg->r, msg->g, msg->b);
  }

  void reset_callback(
    const std::shared_ptr<sabacan_msgs::srv::SabacanReset::Request> request,
    std::shared_ptr<sabacan_msgs::srv::SabacanReset::Response> response)
  {
    (void)request;
    led_init();
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "LED driver has been reset.");
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

  void led_init()
  {
    std::vector<std::string> param_name{
      "enable_auto_transition", "emg_blink_period", "emg_color",   "enable_monitor_period",
      "monitor_period",         "monitor_reg1",     "monitor_reg2"};

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

  bool update_parameters(const rclcpp::Parameter & parameter)
  {
    const std::string & name = parameter.get_name();

    if (name == "enable_auto_transition") {
      enable_auto_transition_ = parameter.as_bool();
      led_driver_->setEnableAutoTransition(enable_auto_transition_);
      delay();
    } else if (name == "emg_blink_period") {
      emg_blink_period_ = parameter.as_int();
      led_driver_->setEmgBlinkPeriod(static_cast<uint16_t>(emg_blink_period_));
      delay();
    } else if (name == "emg_color") {
      std::vector<int64_t> emg_color = parameter.as_integer_array();
      if (check_data_range_and_size<int64_t>(emg_color, 0, 255, M, "emg_color") == false) {
        return false;
      }
      emg_color_ = emg_color;
      for (int m = 0; m < M; m++) {
        led_driver_->setEmgColor(
          m, (uint8_t)(emg_color_[0]), (uint8_t)(emg_color_[1]), (uint8_t)(emg_color_[2]));
        delay();
      }
    } else if (name == "monitor_period") {
      monitor_period_ = parameter.as_int();
      if (enable_monitor_period_) {
        led_driver_->setMonitorPeriod(static_cast<uint16_t>(monitor_period_));
        delay();
      }
    } else if (name == "monitor_reg1") {
      monitor_reg1_ = parameter.as_int();
      led_driver_->setMonitorReg1(static_cast<uint64_t>(monitor_reg1_));
      delay();
    } else if (name == "monitor_reg2") {
      monitor_reg2_ = parameter.as_int();
      led_driver_->setMonitorReg2(static_cast<uint64_t>(monitor_reg2_));
      delay();
    } else if (name == "enable_monitor_period") {
      enable_monitor_period_ = parameter.as_bool();
      if (enable_monitor_period_) {
        led_driver_->setMonitorPeriod(static_cast<uint16_t>(monitor_period_));
        delay();
      } else {
        led_driver_->setMonitorPeriod(0);
        delay();
      }
    }
    return true;
  }

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanLEDRef>::SharedPtr sabacan_led_ref_subscription_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanLEDMode>::SharedPtr sabacan_led_mode_subscription_;
  std::shared_ptr<CanDriver> can_driver_;
  std::shared_ptr<LEDDriver> led_driver_;
  rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  static constexpr int M = 3;
  int64_t board_id_;

  bool enable_auto_transition_;
  int64_t emg_blink_period_;
  std::vector<int64_t> emg_color_ = std::vector<int64_t>(M, 0);
  bool enable_monitor_period_;
  int64_t monitor_period_;
  int64_t monitor_reg1_;
  int64_t monitor_reg2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();

  // options をコンストラクタに渡す
  rclcpp::spin(std::make_shared<SabacanLEDNode>(options));

  rclcpp::shutdown();
  return 0;
}