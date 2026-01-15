cmake_minimum_required(VERSION 3.8)
project(ydlidar_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sdm15_receiver_node src/sdm15_receiver_node.cpp)
ament_target_dependencies(sdm15_receiver_node rclcpp sensor_msgs)
# インクルードディレクトリを設定
target_include_directories(sdm15_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS sdm15_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

# launchディレクトリとその中の全ファイルをインストール
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# paramsディレクトリとその中の全ファイルをインストール
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
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
