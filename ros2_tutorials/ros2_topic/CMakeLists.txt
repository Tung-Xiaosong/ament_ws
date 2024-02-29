cmake_minimum_required(VERSION 3.8)
project(ros2_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(topic_publisher_node src/topic_publisher.cpp)
add_executable(topic_subscriber_node src/topic_subscriber.cpp)

ament_target_dependencies(topic_publisher_node
  rclcpp
  std_msgs)
ament_target_dependencies(topic_subscriber_node
  rclcpp
  std_msgs)

install(TARGETS
  topic_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  topic_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
