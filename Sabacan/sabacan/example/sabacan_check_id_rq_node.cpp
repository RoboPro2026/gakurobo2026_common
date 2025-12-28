/**
 * @file sabacan_check_id_rq_node.cpp
 * @author Yamaguchi Yudai
 * @brief 1秒おきに、ID_RQを送信するプログラム
 * @version 0.1
 * @date 2025-12-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan/sabacan.h"

using namespace std::chrono_literals;

class SabacanCheckIdRqNode : public rclcpp::Node
{
public:
  SabacanCheckIdRqNode() : Node("sabacan_check_id_rq_node")
  {
    can_driver_ = std::make_shared<CanDriver>();
    common_driver_ = std::make_unique<CommonDataDriver>(can_driver_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100,
      std::bind(&SabacanCheckIdRqNode::can_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(1s, std::bind(&SabacanCheckIdRqNode::timer_callback, this));
  }

private:
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
    can_pub_->publish(std::move(msg));
  }

  void timer_callback()
  {
    //　id_rqを送信
    common_driver_->id_rq();
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
      msg->id, msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
      msg->data[5], msg->data[6], msg->data[7]);
    auto frame =
      can_driver_->rx_frame(msg->id, msg->data.data(), msg->dlc, msg->is_rtr, msg->is_extended);
    frame.len = msg->dlc;
    // リクエストを受け取ったデバイスは、Board IDに自身のID（ロータリーDIPの値）、data[0]に基板の形式を入れてData type 0x0で返す。
    if (frame.data_type == DataType::COMMON) {
      if (frame.data[0] == DataType::POWER) {
        RCLCPP_INFO(
          this->get_logger(), "Received data_type = POWER, board_id = %d", frame.board_id);
      } else if (frame.data[0] == DataType::ROBOMAS_V1) {
        RCLCPP_INFO(
          this->get_logger(), "Received data_type = ROBOMAS_V1, board_id = %d", frame.board_id);
      } else if (frame.data[0] == DataType::GPIO) {
        RCLCPP_INFO(this->get_logger(), "Received data_type = GPIO, board_id = %d", frame.board_id);
      } else if (frame.data[0] == DataType::ROBOMAS_V2) {
        RCLCPP_INFO(
          this->get_logger(), "Received data_type = ROBOMAS_V2, board_id = %d", frame.board_id);
      } else if (frame.data[0] == DataType::LED) {
        RCLCPP_INFO(this->get_logger(), "Received data_type = LED, board_id = %d", frame.board_id);
      }
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<CanDriver> can_driver_;
  std::unique_ptr<CommonDataDriver> common_driver_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SabacanCheckIdRqNode>());
  rclcpp::shutdown();
  return 0;
}
