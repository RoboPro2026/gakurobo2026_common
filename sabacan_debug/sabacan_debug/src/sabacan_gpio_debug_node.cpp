#include <rclcpp/rclcpp.hpp>
#include <sabacan_debug_msgs/msg/sabacan_gpio_ref_debug.hpp>
#include <sabacan_debug_msgs/msg/sabacan_gpio_ref_int_debug.hpp>
#include <sabacan_debug_msgs/msg/sabacan_gpio_res_debug.hpp>
#include <sabacan_msgs/msg/sabacan_gpio_ref_float.hpp>
#include <sabacan_msgs/msg/sabacan_gpio_ref_int.hpp>
#include <sabacan_msgs/msg/sabacan_gpio_status.hpp>

using namespace std::placeholders;

class SabacanGPIODebugNode : public rclcpp::Node
{
public:
  SabacanGPIODebugNode() : Node("sabacan_gpio_debug_node")
  {
    // パラメータの宣言
    this->declare_parameter("board_id", 0);
    this->declare_parameter("publish_rate_hz", 10.0);

    // パラメータの取得
    board_id_ = this->get_parameter("board_id").as_int();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    // サブスクライバーの作成
    sabacan_gpio_ref_float_subscriber_ =
      this->create_subscription<sabacan_msgs::msg::SabacanGPIORefFloat>(
        "/sabacan_gpio_ref_float" + std::to_string(board_id_), 1,
        std::bind(&SabacanGPIODebugNode::sabacanGPIORefFloatDebugCallback, this, _1));
    sabacan_gpio_ref_int_subscriber_ =
      this->create_subscription<sabacan_msgs::msg::SabacanGPIORefInt>(
        "/sabacan_gpio_ref_int" + std::to_string(board_id_), 1,
        std::bind(&SabacanGPIODebugNode::sabacanGPIORefIntDebugCallback, this, _1));
    sabacan_gpio_status_subscriber_ =
      this->create_subscription<sabacan_msgs::msg::SabacanGPIOStatus>(
        "/sabacan_gpio_status" + std::to_string(board_id_), 1,
        std::bind(&SabacanGPIODebugNode::sabacanGPIOStatusDebugCallback, this, _1));

    // パブリッシャーの作成
    sabacan_gpio_ref_float_debug_publisher_ =
      this->create_publisher<sabacan_debug_msgs::msg::SabacanGPIORefDebug>(
        "/sabacan_gpio_ref_float_debug" + std::to_string(board_id_), 10);
    sabacan_gpio_ref_int_debug_publisher_ =
      this->create_publisher<sabacan_debug_msgs::msg::SabacanGPIORefIntDebug>(
        "/sabacan_gpio_ref_int_debug" + std::to_string(board_id_), 10);
    sabacan_gpio_status_debug_publisher_ =
      this->create_publisher<sabacan_debug_msgs::msg::SabacanGPIOResDebug>(
        "/sabacan_gpio_status_debug" + std::to_string(board_id_), 10);

    // タイマーの作成
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&SabacanGPIODebugNode::publishTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SabacanGPIODebugNode started");
    RCLCPP_INFO(this->get_logger(), "Board ID: %d", board_id_);
    RCLCPP_INFO(this->get_logger(), "Publish rate: %f Hz", publish_rate_hz_);
  }

private:
  void sabacanGPIORefFloatDebugCallback(const sabacan_msgs::msg::SabacanGPIORefFloat::SharedPtr msg)
  {
    sabacan_gpio_ref_float_debug_array_.ref_array[msg->pin_number] = *msg;
  }
  void sabacanGPIORefIntDebugCallback(const sabacan_msgs::msg::SabacanGPIORefInt::SharedPtr msg)
  {
    sabacan_gpio_ref_int_debug_array_.ref_array[msg->pin_number] = *msg;
  }
  void sabacanGPIOStatusDebugCallback(const sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg)
  {
    sabacan_gpio_status_debug_array_.res_array[msg->pin_number] = *msg;
  }

  void publishTimerCallback()
  {
    sabacan_gpio_ref_float_debug_publisher_->publish(sabacan_gpio_ref_float_debug_array_);
    sabacan_gpio_ref_int_debug_publisher_->publish(sabacan_gpio_ref_int_debug_array_);
    sabacan_gpio_status_debug_publisher_->publish(sabacan_gpio_status_debug_array_);
  }

  rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIORefFloat>::SharedPtr
    sabacan_gpio_ref_float_subscriber_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIORefInt>::SharedPtr
    sabacan_gpio_ref_int_subscriber_;
  rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr
    sabacan_gpio_status_subscriber_;
  rclcpp::Publisher<sabacan_debug_msgs::msg::SabacanGPIORefDebug>::SharedPtr
    sabacan_gpio_ref_float_debug_publisher_;
  rclcpp::Publisher<sabacan_debug_msgs::msg::SabacanGPIORefIntDebug>::SharedPtr
    sabacan_gpio_ref_int_debug_publisher_;
  rclcpp::Publisher<sabacan_debug_msgs::msg::SabacanGPIOResDebug>::SharedPtr
    sabacan_gpio_status_debug_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  int board_id_;
  double publish_rate_hz_;
  sabacan_debug_msgs::msg::SabacanGPIORefDebug sabacan_gpio_ref_float_debug_array_;
  sabacan_debug_msgs::msg::SabacanGPIORefIntDebug sabacan_gpio_ref_int_debug_array_;
  sabacan_debug_msgs::msg::SabacanGPIOResDebug sabacan_gpio_status_debug_array_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SabacanGPIODebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}