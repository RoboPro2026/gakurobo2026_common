#include "sabacan/sabacan_robomasv2_node.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();

  // options をコンストラクタに渡す
  rclcpp::spin(std::make_shared<SabacanRobomasV2Node>(options));

  rclcpp::shutdown();
  return 0;
}
