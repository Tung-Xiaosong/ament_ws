#ifndef RECHARGE_CONTROLLER_H
#define RECHARGE_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "yhs_can_interfaces/msg/iofb.hpp"

namespace recharge_controller_ns
{
    class RechargeController : public rclcpp::Node
    {
        public:
            RechargeController();
            ~RechargeController();
            void start();
            void stop();
        
        private:
            double robot_x_;
    };
}

#endif // RECHARGE_CONTROLLER_H