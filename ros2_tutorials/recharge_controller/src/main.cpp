#include "recharge_controller/recharge_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RechargeController>());
    rclcpp::shutdown();
    return 0;
}