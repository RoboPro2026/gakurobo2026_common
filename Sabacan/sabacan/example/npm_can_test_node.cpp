/**
 * @file npm_uart_test_node.cpp
 * @author Yamaguchi Yudai
 * @brief RobomasV2のシーケンス番号(ID:NOP)を読んで、パケットロスがないことを確認するノード
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/*
実行方法
# ros2_socketcanは別ターミナルで起動しておくこと
# 実行、board_idは実験対象のロボマス制御基板
ros2 run sabacan npm_can_test_node --ros-args -p board_id:=2

解説
PACKET_SIZE回分のシーケンス番号を保存し、最後のデータを受信したときにパケットロスをチェックする。
シーケンス番号が連続していなければパケットロスとカウントし、受信成功率を計算して表示する。
もし、2秒間データが受信できなければエラーメッセージを表示する。
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

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("npm_can_test_node")
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
        "Mandatory parameter 'board_id' was not set. Please provide it at launch");
      // コンストラクタで例外を投げることで、ノードの初期化を安全に中止します
      throw;
    }

    can_driver_ = std::make_shared<CanDriver>();
    robomas_driver_ = std::make_shared<RobomasDriverV2>(can_driver_, (int)board_id_);

    can_driver_->register_tx_callback(
      [this](uint32_t id, uint8_t * data, uint8_t dlc, bool is_remote, bool is_ext) {
        this->tx(id, data, dlc, is_remote, is_ext);
      });

    can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100, std::bind(&MyNode::can_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));

    // ロボマス0番目のみ、NOPのmonitor_regを有効にする
    uint64_t reg = 1 << RobomasV2::NOP;
    robomas_driver_->setMonitorReg1(0, reg);
    // monitro_periodを10msに設定
    robomas_driver_->setMonitorPeriod(1);
    RCLCPP_INFO(this->get_logger(), "program start");
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
    // if ((this->now() - last_time_).seconds() > 2.0) {
    //   RCLCPP_ERROR(this->get_logger(), "No data received few second.");
    // }
    if ((this->now() - start_time_).seconds() > 10.0) {
      int loss_cnt = 0;
      for (int i = 1; i < rx_data.size(); i++) {
        // シーケンス番号が連続していなければ、パケットロスとカウントする
        if (rx_data[i] != rx_data[i - 1] + 1) {
          loss_cnt++;
        }
      }
      // 受信成功率を表示
      if (rx_data.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "No data received");
        exit(0);
      }
      double success_rate = (double)(rx_data.size() - loss_cnt) / (double)rx_data.size() * 100.0;
      RCLCPP_INFO(
        this->get_logger(), "CAN Received %d packets, Loss: %d packets, Success rate: %.2f%%",
        rx_data.size(), loss_cnt, success_rate);
      exit(0);
    }
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "Received CAN frame: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
    //   msg->id, msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
    //   msg->data[5], msg->data[6], msg->data[7]);
    auto frame =
      can_driver_->rx_frame(msg->id, msg->data.data(), msg->dlc, msg->is_rtr, msg->is_extended);

    // RobomasV2のNOPレジスタだった場合はカウンタを更新
    bool is_robomas_nop = frame.data_type == DataType::ROBOMAS_V2 &&
                          frame.board_id == (uint8_t)board_id_ &&
                          (frame.register_id & 0xFF) == RobomasV2::NOP;
    bool is_normal_nop = (msg->id == 0);
    if (is_robomas_nop || is_normal_nop) {
      // motor_numが0でなければ処理しない
      int motor_num = frame.register_id >> 8;
      if (motor_num != 0) {
        return;
      }
      rx_data.push_back(
        frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
      /*
      // 最終受信時刻を更新
      last_time_ = this->now();
      // データを更新
      rx_data[index] =
        frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24);

      // 配列の最後のデータのときはパケットロスをチェックし、受信成功率を計算
      if (index == PACKET_SIZE - 1) {
        int loss_cnt = 0;
        for (int i = 1; i < PACKET_SIZE; i++) {
          // シーケンス番号が連続していなければ、パケットロスとカウントする
          if (rx_data[i] != rx_data[i - 1] + 1) {
            loss_cnt++;
          }
        }
        // 受信成功率を表示
        double success_rate = (double)(PACKET_SIZE - loss_cnt) / PACKET_SIZE * 100.0;
        RCLCPP_INFO(
          this->get_logger(), "CAN Received %d packets, Loss: %d packets, Success rate: %.2f%%",
          PACKET_SIZE, loss_cnt, success_rate);
      }
      // インデックスを更新
      index = (index + 1) % PACKET_SIZE;
      // RCLCPP_INFO(this->get_logger(), "NOP Index: %d", rx_data[index - 1]);
      */
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<CanDriver> can_driver_;
  std::shared_ptr<RobomasDriverV2> robomas_driver_;
  int64_t board_id_;
  rclcpp::Time last_time_ = this->now();
  rclcpp::Time start_time_ = this->now();

  // int PACKET_SIZE = 100;
  // std::vector<int> rx_data = std::vector<int>(PACKET_SIZE, -1);
  std::vector<int> rx_data;
  int index = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}