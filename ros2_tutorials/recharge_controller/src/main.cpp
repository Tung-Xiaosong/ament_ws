#include "recharge_controller/recharge_controller.hpp"

int main(int argc, char** argv)
{
  // 初始化ROS节点
  rclcpp::init(argc, argv);

  // 创建RechargeController对象
  auto recharge_controller = std::make_shared<recharge_controller_ns::RechargeController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(recharge_controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}