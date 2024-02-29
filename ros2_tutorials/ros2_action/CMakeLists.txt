cmake_minimum_required(VERSION 3.8)
project(ros2_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_tutorials_interfaces REQUIRED)

add_executable(action_server_geometric_sequence src/geometric_action_server.cpp)#这里要在.cpp前写上src/，否则会报错：找不到源文件
ament_target_dependencies(action_server_geometric_sequence
  "rclcpp"
  "rclcpp_action"
  "ros2_tutorials_interfaces")

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

install(TARGETS
  action_server_geometric_sequence
  DESTINATION lib/${PROJECT_NAME})

ament_package()
