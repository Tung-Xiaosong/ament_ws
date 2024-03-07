#include "recharge_controller/recharge_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recharge Controller Started !");
    rclcpp::spin(std::make_shared<RechargeController>());
    rclcpp::shutdown();
    return 0;
}