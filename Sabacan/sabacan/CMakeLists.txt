cmake_minimum_required(VERSION 3.8)
project(sabacan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sabacan_msgs REQUIRED)

# Add executable
add_executable(sabacan_check_id_rq_node example/sabacan_check_id_rq_node.cpp)
add_executable(sabacan_gpio_node src/sabacan_gpio_node.cpp)
add_executable(sabacan_power_node src/sabacan_power_node.cpp)
add_executable(sabacan_robomas_node src/sabacan_robomas_node.cpp)
add_executable(sabacan_robomasv2_node src/sabacan_robomasv2_node.cpp)

ament_target_dependencies(sabacan_check_id_rq_node rclcpp can_msgs std_msgs sabacan_msgs)
ament_target_dependencies(sabacan_gpio_node rclcpp can_msgs std_msgs sabacan_msgs)
ament_target_dependencies(sabacan_power_node rclcpp can_msgs std_msgs sabacan_msgs)
ament_target_dependencies(sabacan_robomas_node rclcpp can_msgs std_msgs sabacan_msgs)
ament_target_dependencies(sabacan_robomasv2_node rclcpp can_msgs std_msgs sabacan_msgs)

# インクルードディレクトリを設定
target_include_directories(sabacan_check_id_rq_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sabacan_gpio_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sabacan_power_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sabacan_robomas_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sabacan_robomasv2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  sabacan_check_id_rq_node
  sabacan_gpio_node
  sabacan_power_node
  sabacan_robomas_node
  sabacan_robomasv2_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_can_device_robomas_driver_v2 test/test_can_device_robomas_driver_v2.cpp)
  ament_add_gtest(test_sabacan_robomasv2_node test/test_sabacan_robomasv2_node.cpp)

  ament_target_dependencies(test_can_device_robomas_driver_v2
    rclcpp
    can_msgs
    std_msgs
    sabacan_msgs
    ament_cmake_gtest
    rcutils # ログレベル設定用
  )

  ament_target_dependencies(test_sabacan_robomasv2_node
    rclcpp
    can_msgs
    std_msgs
    sabacan_msgs
    ament_cmake_gtest
    rcutils # ログレベル設定用
  )

  target_include_directories(test_can_device_robomas_driver_v2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_include_directories(test_sabacan_robomasv2_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )


endif()

ament_package()
