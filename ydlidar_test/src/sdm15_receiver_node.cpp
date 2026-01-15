#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

class Sdm15ReceiverNode : public rclcpp::Node
{
public:
  Sdm15ReceiverNode() : Node("sdm15_receiver_node")
  {
    // 受信ノードを作成
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Sdm15ReceiverNode::sdm15_receive_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SDM15 Receiver Node has been started.");
  }

private:
  /**
   * @brief 受信コールバック
   * 
   * @param msg 
   */
  void sdm15_receive_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received data: %.3f", msg->ranges[4]);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sdm15ReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
