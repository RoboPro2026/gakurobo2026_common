/**
 * @file test_sabacan_robomasv2_node.cpp
 * @author Yamaguchi Yudai
 * @brief sabacan_robomasv2_nodeのテスト用ノード
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// TODO: やる気がなくなって、中途半端で止まっている状態なので、気が向いたら実装を完成させる

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstring>  // memcpy
#include <future>   // std::promise, std::future
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "can_msgs/msg/frame.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/node_options.hpp"
#include "rcutils/logging.h"     // グローバルログレベル設定用
#include "sabacan/can_define.h"  // CAN ID定義
#include "sabacan/sabacan_robomasv2_node.h"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_msgs/srv/set_robomas_gains.hpp"

using namespace std::chrono_literals;

std::shared_ptr<SabacanRobomasV2Node> target_node = nullptr;
std::shared_ptr<rclcpp::Node> test_node = nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
std::thread spin_thread;
std::shared_ptr<CanDriver> can_driver;
// std::mutex<std::vector<CanFrame>> received_can_frame;
rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher;
rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription;
rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr sabacan_ref_publisher;
int board_id = 0;

const std::string TARGET_NODE_NAME = "sabacan_robomasv2_node";
//
const std::string STATUS_TOPIC = "/sabacan_robomas_status" + std::to_string(board_id);
//
const std::string REF_TOPIC = "/sabacan_robomas_ref" + std::to_string(board_id);
//
const std::string SET_GAINS_SERVICE = "/set_robomas_gains";
//
const std::string RESET_SERVICE = "/sabacan_robomas_reset";
//
const std::string TO_CAN_TOPIC = "/to_can_bus";
//
const std::string FROM_CAN_TOPIC = "/from_can_bus";

class MyTestNode : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // ROS 2 初期化
    rclcpp::init(0, nullptr);

    // sabacan_robomasv2_node のログレベルを FATAL に設定
    rcutils_logging_set_logger_level("sabacan_robomasv2_node", RCUTILS_LOG_SEVERITY_ERROR);
    // NodeOptions を作成
    // TODO: board_idは乱数で設定するようにする。
    auto target_node_options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("board_id", board_id)  // board_id をパラメータで渡す
    });

    target_node = std::make_shared<SabacanRobomasV2Node>(target_node_options);
    test_node = std::make_shared<rclcpp::Node>("test_sabacan_robomasv2_node");
    // INFO, WARN, ERROR ログを抑制するため、ログレベルを FATAL に設定
    test_node->get_logger().set_level(rclcpp::Logger::Level::Fatal);

    /*
    can_subscription = test_node->create_subscription<can_msgs::msg::Frame>(
      TO_CAN_TOPIC, 10, [&](const can_msgs::msg::Frame::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(received_can_frame);
        CanFrame frame;
        frame.id = msg->id;
        frame.dlc = msg->dlc;
        std::memcpy(frame.data, msg->data.data(), msg->dlc);
        frame.is_extended_id = msg->is_extended;
        frame.is_remote_frame = msg->is_rtr;
        received_can_frame.push_back(frame);
      });
    */

    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(target_node);
    exec->add_node(test_node);
    spin_thread = std::thread([&]() { exec->spin(); });

    std::this_thread::sleep_for(1s);
  }

  static void TearDownTestSuite()
  {
    if (exec) {
      exec->cancel();
    }
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    target_node.reset();
    test_node.reset();
    exec.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down test node...");
    rclcpp::shutdown();
  }
};

// ノードが存在しているかの確認
TEST_F(MyTestNode, NodeExists)
{
  RCLCPP_INFO(target_node->get_logger(), "Checking if target node exists...");
  // SetUpTestSuiteが成功していればOK
  EXPECT_TRUE(target_node != nullptr);
  EXPECT_TRUE(test_node != nullptr);
}

// board_idパラメータの確認
TEST_F(MyTestNode, ParameterBoardId)
{
  // 本当はノード起動時にいろいろなboard_idで起動して確認したいが、面倒なので行わない
  RCLCPP_INFO(target_node->get_logger(), "Checking 'board_id' parameter...");
  int64_t param_board_id;
  bool ret = target_node->get_parameter("board_id", param_board_id);
  EXPECT_TRUE(ret);
  EXPECT_EQ(param_board_id, board_id);

  // board_idは変更不可であることを確認
  int64_t new_board_id = (board_id + 1) % 10;
  EXPECT_FALSE(target_node->set_parameter(rclcpp::Parameter("board_id", new_board_id)).successful);
  // board_idが書き換わっていないことを確認
  target_node->get_parameter("board_id", param_board_id);
  EXPECT_EQ(param_board_id, board_id);

  // 範囲外の値を設定したときも失敗することを確認
  EXPECT_FALSE(target_node->set_parameter(rclcpp::Parameter("board_id", -1)).successful);
  EXPECT_FALSE(target_node->set_parameter(rclcpp::Parameter("board_id", 10)).successful);
}

/**
 * @brief SetGains サービス呼び出しのテスト
 */
TEST_F(MyTestNode, SetGainsServiceCall)
{
  RCLCPP_INFO(test_node->get_logger(), "Testing SetGains service call...");
  auto client = test_node->create_client<sabacan_msgs::srv::SetRobomasGains>(SET_GAINS_SERVICE);

  ASSERT_TRUE(client->wait_for_service(2s)) << "SetGains service not available";

  auto request = std::make_shared<sabacan_msgs::srv::SetRobomasGains::Request>();
  request->motor_number = 1;
  request->set_speed_gains = true;
  request->speed_gain_p = 1.2f;
  request->speed_gain_i = 0.8f;
  request->torque_lim = 5.0f;

  // サービスを非同期で呼び出し
  auto future = client->async_send_request(request);

  // 応答が返るまで待機
  auto future_status = future.wait_for(5s);
  ASSERT_EQ(future_status, std::future_status::ready) << "SetGains service call timed out";

  auto response = future.get();
  ASSERT_TRUE(response->success) << "SetGains service failed with message: " << response->message;
  RCLCPP_INFO(test_node->get_logger(), "SetGains Response: %s", response->message.c_str());
}

// TEST_F(MytestNode, SabacanRobomasRef)
// {
//   std::vector<can_msgs::Frame> receive_data;
//   auto can_sub = test_node->create_subscription<can_msgs_::msg::Frame>(
//     TO_CAN_TOPIC, 10,
//     [&receive_data](const can_msgs::Frame::SharedPtr msg) { receive_data.push_back(msg); });
// }

/**
 * @brief /sabacan_robomas_ref (指令値) -> /to_can_bus (CAN送信) のテスト
 *
 * 1. /to_can_bus を購読 (待機)
 * 2. /sabacan_robomas_ref に指令値を送信 (トリガー)
 * 3. /to_can_bus から期待通りのCANフレームが送信されたか検証
 */
TEST_F(MyTestNode, RefToCanPublish)
{
  RCLCPP_INFO(test_node->get_logger(), "Testing Ref -> CAN publish...");

  // --- 1. 待機 (CANフレーム受信用) ---
  std::promise<can_msgs::msg::Frame> can_promise;
  auto can_future = can_promise.get_future();

  // test_node で /to_can_bus を購読
  auto can_sub = test_node->create_subscription<can_msgs::msg::Frame>(
    TO_CAN_TOPIC, 10, [&can_promise](const can_msgs::msg::Frame::SharedPtr msg) {
      // メッセージを受信したら Promise を満たす
      try {
        can_promise.set_value(*msg);
      } catch (const std::future_error &) {
        // 既にセットされていたら無視
      }
    });

  // --- 2. トリガー (指令値送信用) ---
  auto ref_pub = test_node->create_publisher<sabacan_msgs::msg::SabacanRobomasRef>(REF_TOPIC, 10);

  // Publisher/Subscriber が接続されるのを少し待つ
  std::this_thread::sleep_for(200ms);

  // --- 3. 指令値を送信 ---
  auto ref_msg = std::make_shared<sabacan_msgs::msg::SabacanRobomasRef>();
  ref_msg->motor_number = 1;  // モーター番号 1
  ref_msg->ref = 3.14f;       // 指令値 (速度 3.14 rad/s)

  // ノードのデフォルトパラメータでは motor 1 は "VELOCITY" モード
  // よって、sabacan_ref_callback により
  // robomas_driver_->setSpeedTarget(1, 3.14f) が呼ばれるはず
  ref_pub->publish(*ref_msg);

  // --- 4. 結果を待機 ---
  // SetUpTestSuite で別スレッド (spin_thread) がノードをスピンさせているため、
  // rclcpp::spin_until_future_complete は使わず、future の wait_for を使う
  auto future_status = can_future.wait_for(3s);  // 3秒タイムアウト

  // --- 5. 検証 ---
  ASSERT_EQ(future_status, std::future_status::ready)
    << "Timeout: Did not receive a CAN frame on " << TO_CAN_TOPIC;

  if (future_status == std::future_status::ready) {
    auto received_frame = can_future.get();

    // 期待されるCAN IDを計算
    // RobomasDriverV2::setSpeedTarget(1, ...)
    // -> tx((1 << 8) | RobomasV2::SPD_TARGET, ...)
    // Register ID = 0x100 | 0x21 = 0x121
    uint32_t expected_register_id = (1 << 8) | RobomasV2::SPD_TARGET;
    // CanDevice::tx_frame
    // ID = (DataType::ROBOMAS_V2=4 << 20) | (board_id=0 << 16) | (register_id=0x121)
    uint32_t expected_can_id =
      (DataType::ROBOMAS_V2 << 20) | (board_id << 16) | expected_register_id;
    // 0x400121

    EXPECT_EQ(received_frame.id, expected_can_id);
    EXPECT_TRUE(received_frame.is_extended);
    EXPECT_EQ(received_frame.dlc, sizeof(float));  // 4 bytes

    // 期待されるデータ (float 3.14f) を検証
    float received_data;
    std::memcpy(&received_data, received_frame.data.data(), sizeof(float));
    EXPECT_NEAR(received_data, 3.14f, 1e-5);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}