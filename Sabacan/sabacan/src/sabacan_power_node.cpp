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
#include "sabacan_msgs/srv/sabacan_reset.hpp"

using namespace std::chrono_literals;

class SabacanPowerNode : public rclcpp::Node
{
public:
  SabacanPowerNode() : Node("sabacan_power_node")
  {
    can_driver_ = std::make_shared<CanDriver>();
    common_data_driver_ = std::make_unique<CommonDataDriver>(can_driver_);
    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    // CANデータ送信用のPublisher
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);

    // 指令値を受け取るSubscriber
    sabacan_ref_subscription_ = this->create_subscription<sabacan_msgs::msg::SabacanPowerRef>(
      "/sabacan_power_ref", 10,
      std::bind(&SabacanPowerNode::sabacan_ref_callback, this, std::placeholders::_1));

    // 非常停止信号用
    // 非常停止信号はなんらかの理由があって読めなかったら嫌なので、一定周期で送る
    // timer_ = this->create_wall_timer(50ms, std::bind(&SabacanPowerNode::timer_callback, this));

    // 非常停止信号の初期値はfalseに設定。
    // falseに設定する理由は、初期化などがうまく行かなかったら面倒だから。
    is_ems_ = false;
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
    RCLCPP_INFO(this->get_logger(), "Sending CAN frame: ID=0x%X", msg->id);
    can_publisher_->publish(std::move(msg));
  }

  /**
   * @brief トピックで目標値が更新されたときのcallback
   *
   * @param msg
   */
  void sabacan_ref_callback(const sabacan_msgs::msg::SabacanPowerRef::SharedPtr msg)
  {
    is_ems_ = msg->is_ems;
    if (is_ems_)
      common_data_driver_->ems();
    else
      common_data_driver_->reset_ems();
    RCLCPP_INFO(
      this->get_logger(), "Received Emergency signal: is_ems = %s",
      is_ems_ == true ? "true" : "false");
  }

  void timer_callback()
  {
    if (is_ems_)
      common_data_driver_->ems();
    else
      common_data_driver_->reset_ems();
    RCLCPP_INFO(
      this->get_logger(), "Send Emergency signal: is_ems = %s", is_ems_ ? "true" : "false");
  }

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanPowerRef>::SharedPtr sabacan_ref_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<CanDriver> can_driver_;
  std::unique_ptr<CommonDataDriver> common_data_driver_;
  bool is_ems_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SabacanPowerNode>());
  rclcpp::shutdown();
  return 0;
}