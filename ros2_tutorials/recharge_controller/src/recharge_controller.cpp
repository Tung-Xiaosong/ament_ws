#include "recharge_controller/recharge_controller.hpp"

namespace recharge_controller_ns
{
    // RechargeController::RechargeController()
    // {

    // }
    // RechargeController::~RechargeController()
    // {

    // }
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<recharge_controller_ns::RechargeController>());//这里用spin 类必须要继承Node
    rclcpp::shutdown();
    return 0;
}
}