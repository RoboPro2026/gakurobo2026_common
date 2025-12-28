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
#include "sabacan_msgs/msg/sabacan_power_ref.hpp"
#include "sabacan_msgs/msg/sabacan_power_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"

using namespace std::chrono_literals;

class SabacanPowerNode : public rclcpp::Node
{
public:
  SabacanPowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sabacan_power_node")
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

    this->declare_parameter<int64_t>("cell_n", 6);
    this->declare_parameter<int64_t>("ex_ems_trg", 0);
    this->declare_parameter<bool>("common_ems_en", true);
    this->declare_parameter<double>("v_limit_high", 4.3);
    this->declare_parameter<double>("v_limit_low", 3.7);
    this->declare_parameter<double>("i_limit", 100.0);
    // デフォルトでは、PCU_STATE、OUT_V、OUT_Iを20Hzでモニタリング
    this->declare_parameter("enable_monitor_period", true);
    this->declare_parameter("monitor_period", 50);
    int64_t reg = 0LL;
    reg |= 1LL << Power::PCU_STATE;
    reg |= 1LL << Power::OUT_V;
    reg |= 1LL << Power::OUT_I;
    this->declare_parameter("monitor_reg", reg);

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);

    // CANデータ受信用のSubscriber
    can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100,
      std::bind(&SabacanPowerNode::can_callback, this, std::placeholders::_1));

    sabacan_power_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanPowerRef>(
      "/sabacan_power_ref" + std::to_string(board_id_), 10,
      std::bind(&SabacanPowerNode::sabacan_power_ref_callback, this, std::placeholders::_1));

    sabacan_power_status_publisher_ = this->create_publisher<sabacan_msgs::msg::SabacanPowerStatus>(
      "/sabacan_power_status" + std::to_string(board_id_), 10);

    can_driver_ = std::make_shared<CanDriver>();
    common_data_driver_ = std::make_shared<CommonDataDriver>(can_driver_);
    power_driver_ = std::make_shared<PowerDriver>(can_driver_, board_id_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    // 100Hzのタイマーを設定
    timer_ = this->create_wall_timer(10ms, std::bind(&SabacanPowerNode::timer_callback, this));

    power_init();
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
    power_driver_->receive(frame);
  }

  /**
   * @brief トピックで目標値が更新されたときのcallback
   *
   * @param msg
   */
  void sabacan_power_ref_callback(const sabacan_msgs::msg::SabacanPowerRef::SharedPtr msg)
  {
    bool is_ems = msg->is_ems;
    if (is_ems)
      common_data_driver_->ems();
    else
      common_data_driver_->reset_ems();
    RCLCPP_INFO(
      this->get_logger(), "Received Emergency signal: is_ems = %s",
      is_ems == true ? "true" : "false");
  }

  void timer_callback()
  {
    // 定期的にSabacanPowerStatusを送信
    sabacan_msgs::msg::SabacanPowerStatus msg;
    msg.pcu_state = power_driver_->pcu_state;
    msg.out_v = power_driver_->out_v;
    msg.out_i = power_driver_->out_i;
    sabacan_power_status_publisher_->publish(msg);
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

  void power_init()
  {
    std::vector<std::string> param_name{"cell_n",         "ex_ems_trg",  "common_ems_en",
                                        "v_limit_high",   "v_limit_low", "i_limit",
                                        "monitor_period", "monitor_reg", "enable_monitor_period"};

    rclcpp::Time start_time = this->get_clock()->now();

    for (size_t i = 0; i < param_name.size(); i++) {
      // update_parameters関数内ではdelayが入っている。
      update_parameters(this->get_parameter(param_name[i]));
    }

    rclcpp::Time end_time = this->get_clock()->now();
    RCLCPP_INFO(
      this->get_logger(), "Initialization completed in %.3f seconds",
      (end_time - start_time).seconds());
  }

  void delay() { std::this_thread::sleep_for(10ms); }

  bool update_parameters(const rclcpp::Parameter & parameter)
  {
    const std::string & name = parameter.get_name();

    if (name == "cell_n") {
      cell_n_ = parameter.as_int();
      power_driver_->setCellN(cell_n_);
      delay();
    } else if (name == "ex_ems_trg") {
      ex_ems_trg_ = parameter.as_int();
      power_driver_->setExEmsTrg(ex_ems_trg_);
      delay();
    } else if (name == "common_ems_en") {
      common_ems_en_ = parameter.as_bool();
      power_driver_->setCommonEmsEn(common_ems_en_);
      delay();
    } else if (name == "v_limit_high") {
      v_limit_high_ = parameter.as_double();
      power_driver_->setVLimitHigh(v_limit_high_);
      delay();
    } else if (name == "v_limit_low") {
      v_limit_low_ = parameter.as_double();
      power_driver_->setVLimitLow(v_limit_low_);
      delay();
    } else if (name == "i_limit") {
      i_limit_ = parameter.as_double();
      power_driver_->setILimit(i_limit_);
      delay();
    } else if (name == "monitor_period") {
      monitor_period_ = parameter.as_int();
      power_driver_->setMonitorPeriod(monitor_period_);
      delay();
    } else if (name == "monitor_reg") {
      monitor_reg_ = parameter.as_int();
      power_driver_->setMonitorReg(monitor_reg_);
      delay();
    } else if (name == "enable_monitor_period") {
      enable_monitor_period_ = parameter.as_bool();
      if (enable_monitor_period_) {
        power_driver_->setMonitorPeriod(monitor_period_);
        delay();
        power_driver_->setMonitorReg(monitor_reg_);
      } else {
        power_driver_->setMonitorPeriod(0);
        delay();
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown parameter: %s", name.c_str());
      return false;
    }
    return true;
  }

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanPowerRef>::SharedPtr
    sabacan_power_ref_subscription_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanPowerStatus>::SharedPtr
    sabacan_power_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<CanDriver> can_driver_;
  std::shared_ptr<PowerDriver> power_driver_;
  std::shared_ptr<CommonDataDriver> common_data_driver_;
  rclcpp::Service<sabacan_msgs::srv::SabacanReset>::SharedPtr reset_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  int64_t board_id_;
  int cell_n_;
  int ex_ems_trg_;
  bool common_ems_en_;
  double v_limit_high_;
  double v_limit_low_;
  double i_limit_;
  bool enable_monitor_period_;
  int monitor_period_;
  int64_t monitor_reg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<SabacanPowerNode>(options));
  rclcpp::shutdown();
  return 0;
}