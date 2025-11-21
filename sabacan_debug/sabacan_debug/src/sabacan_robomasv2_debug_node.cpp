#include <rclcpp/rclcpp.hpp>
#include <sabacan_debug_msgs/msg/sabacan_robomas_ref_debug.hpp>
#include <sabacan_debug_msgs/msg/sabacan_robomas_status_debug.hpp>

using namespace std::placeholders;

class SabacanRobomasV2DebugNode : public rclcpp::Node
{
public:
  SabacanRobomasV2DebugNode() : Node("sabacan_robomasv2_debug_node")
  {
    // パラメータの宣言
    this->declare_parameter("board_id", 0);
    this->declare_parameter("publish_rate_hz", 10.0);

    // パラメータの取得
    board_id_ = this->get_parameter("board_id").as_int();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    // サブスクライバーの作成
    sabacan_robomas_ref_subscriber_ =
      this->create_subscription<sabacan_msgs::msg::SabacanRobomasRef>(
        "/sabacan_robomas_ref" + std::to_string(board_id_), 1,
        std::bind(&SabacanRobomasV2DebugNode::sabacanRobomasRefDebugCallback, this, _1));
    sabacan_robomas_status_subscriber_ =
      this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
        "/sabacan_robomas_status" + std::to_string(board_id_), 1,
        std::bind(&SabacanRobomasV2DebugNode::sabacanRobomasStatusDebugCallback, this, _1));

    // パブリッシャーの作成
    sabacan_robomas_ref_debug_publisher_ =
      this->create_publisher<sabacan_debug_msgs::msg::SabacanRobomasRefDebug>(
        "/sabacan_robomas_ref_debug" + std::to_string(board_id_), 10);
    sabacan_robomas_status_debug_publisher_ =
      this->create_publisher<sabacan_debug_msgs::msg::SabacanRobomasStatusDebug>(
        "/sabacan_robomas_status_debug" + std::to_string(board_id_), 10);

    // タイマーの作成
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&SabacanRobomasV2DebugNode::publishTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SabacanRobomasV2DebugNode started");
    RCLCPP_INFO(this->get_logger(), "Board ID: %d", board_id_);
    RCLCPP_INFO(this->get_logger(), "Publish rate: %f Hz", publish_rate_hz_);
  }

private:
  void sabacanRobomasRefDebugCallback(const sabacan_msgs::msg::SabacanRobomasRef::SharedPtr msg)
  {
    sabacan_robomas_ref_debug_array_.ref_array[msg->motor_number] = *msg;
  }
  void sabacanRobomasStatusDebugCallback(
    const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    sabacan_robomas_status_debug_array_.status_array[msg->motor_number] = *msg;
  }

  void publishTimerCallback()
  {
    sabacan_robomas_ref_debug_publisher_->publish(sabacan_robomas_ref_debug_array_);
    sabacan_robomas_status_debug_publisher_->publish(sabacan_robomas_status_debug_array_);
  }

  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasRef>::SharedPtr
    sabacan_robomas_ref_subscriber_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr
    sabacan_robomas_status_subscriber_;
  rclcpp::Publisher<sabacan_debug_msgs::msg::SabacanRobomasRefDebug>::SharedPtr
    sabacan_robomas_ref_debug_publisher_;
  rclcpp::Publisher<sabacan_debug_msgs::msg::SabacanRobomasStatusDebug>::SharedPtr
    sabacan_robomas_status_debug_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  int board_id_;
  double publish_rate_hz_;
  sabacan_debug_msgs::msg::SabacanRobomasRefDebug sabacan_robomas_ref_debug_array_;
  sabacan_debug_msgs::msg::SabacanRobomasStatusDebug sabacan_robomas_status_debug_array_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SabacanRobomasV2DebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}