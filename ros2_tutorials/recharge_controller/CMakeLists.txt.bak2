cmake_minimum_required(VERSION 3.8)
project(recharge_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yhs_can_interfaces REQUIRED)

# dxs add --------------------

include_directories(
  include
  # ${ament_cmake_INCLUDE_DIRS}
  # ${rclcpp_INCLUDE_DIRS}
  # ${geometry_msgs_INCLUDE_DIRS}
  # ${yhs_can_msgs_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/recharge_controller.cpp
# )

add_executable(recharge_controller_node
  src/recharge_controller.cpp
)

ament_target_dependencies(recharge_controller_node
  rclcpp
  geometry_msgs
  tf2_msgs
  yhs_can_interfaces
  
)

# target_link_libraries(recharge_controller_node
#   ${PROJECT_NAME}
#   # ${ament_cmake_LIBRARIES}
#   # ${rclcpp_LIBRARIES}
#   # ${geometry_msgs_LIBRARIES}
#   # ${nav_msgs_LIBRARIES}
#   # ${tf2_LIBRARIES}
#   # ${tf2_ros_LIBRARIES}
#   # ${actionlib_LIBRARIES}
#   # ${move_base_msgs_LIBRARIES}
#   #${yhs_can_interfaces_LIBRARIES}
# )

install(TARGETS recharge_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

# -------------------

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
