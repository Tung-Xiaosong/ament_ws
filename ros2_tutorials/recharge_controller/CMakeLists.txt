cmake_minimum_required(VERSION 3.5)
project(recharge_controller)

# Compile as C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yhs_can_interfaces REQUIRED)
# add ---
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/recharge_controller.cpp
)

set(dependencies
  rclcpp
  geometry_msgs
  tf2_msgs
  yhs_can_interfaces
  # add ---
  tf2_ros
  nav_msgs
  std_msgs
  rclcpp_action
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()