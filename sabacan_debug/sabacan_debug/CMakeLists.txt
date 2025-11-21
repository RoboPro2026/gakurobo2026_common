cmake_minimum_required(VERSION 3.8)
project(sabacan_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sabacan_msgs REQUIRED)
find_package(sabacan_debug_msgs REQUIRED)

add_executable(sabacan_gpio_debug_node src/sabacan_gpio_debug_node.cpp)
add_executable(sabacan_robomasv2_debug_node src/sabacan_robomasv2_debug_node.cpp)

ament_target_dependencies(sabacan_gpio_debug_node rclcpp can_msgs std_msgs sabacan_msgs sabacan_debug_msgs)
ament_target_dependencies(sabacan_robomasv2_debug_node rclcpp can_msgs std_msgs sabacan_msgs sabacan_debug_msgs)

install(TARGETS
  sabacan_gpio_debug_node
  sabacan_robomasv2_debug_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
