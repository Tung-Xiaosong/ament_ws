cmake_minimum_required(VERSION 3.8)
project(yhs_can_interfaces)

if(CMAKECOMPILER_IS_GNUCXX OR CMAKECXXCOMPILER_ID MATCHES "Clang")
  addCompile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  #DGT
  "msg/DgtBmsFb.msg"
  "msg/DgtBmsFlagFb.msg"
  "msg/DgtChassisInfoFb.msg"
  "msg/DgtCtrlCmd.msg"
  "msg/DgtCtrlFb.msg"
  "msg/DgtFreeCtrlCmd.msg"
  "msg/DgtIoCmd.msg"
  "msg/DgtIoFb.msg"
  "msg/DgtLWheelFb.msg"
  "msg/DgtRWheelFb.msg"
  #FW
  "msg/FwBmsFb.msg"
  "msg/FwBmsFlagFb.msg"
  "msg/FwChassisInfoFb.msg"
  "msg/FwCtrlCmd.msg"
  "msg/FwCtrlFb.msg"
  "msg/FwFrontAngleFb.msg"
  "msg/FwIoCmd.msg"
  "msg/FwIoFb.msg"
  "msg/FwLfWheelFb.msg"
  "msg/FwLrWheelFb.msg"
  "msg/FwRearAngleFb.msg"
  "msg/FwRfWheelFb.msg"
  "msg/FwRrWheelFb.msg"
  #FR
  "msg/FrBmsFlagInfoFb.msg"
  "msg/FrBmsInfoFb.msg"
  "msg/FrChassisInfoFb.msg"
  "msg/FrCtrlCmd.msg"
  "msg/FrCtrlFb.msg"
  "msg/FrDriveMcuEcodeFb.msg"
  "msg/FrIoCmd.msg"
  "msg/FrIoFb.msg"
  "msg/FrLrWheelFb.msg"
  "msg/FrRrWheelFb.msg"
  "msg/FrVehDiagFb.msg"
  #MK
  "msg/MkBmsFlagInfoFb.msg"
  "msg/MkBmsInfoFb.msg"
  "msg/MkChassisInfoFb.msg"
  "msg/MkCtrlCmd.msg"
  "msg/MkCtrlFb.msg"
  "msg/MkDriveMcuEcodeFb.msg"
  "msg/MkIoCmd.msg"
  "msg/MkIoFb.msg"
  "msg/MkLrWheelFb.msg"
  "msg/MkRrWheelFb.msg"
  "msg/MkVehDiagFb.msg"

  #srv
  "srv/DisRecharge.srv"
  "srv/Recharge.srv"
  
  #action
  "action/RecordPath.action"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(amentCmakeCopyrightFOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(amentCmakeCpplintFOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

install(DIRECTORY include/
   DESTINATION include/
)
    
ament_export_include_directories(include)

ament_package()
