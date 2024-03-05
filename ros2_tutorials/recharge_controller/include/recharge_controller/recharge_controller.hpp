#ifndef RECHARGE_CONTROLLER_H
#define RECHARGE_CONTROLLER_H

#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yhs_can_interfaces/srv/recharge.hpp"
#include "yhs_can_interfaces/srv/dis_recharge.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"

// dxs add ------------------------
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


// --------------------------
#include <time.h> // for dxscpp

using namespace Eigen;


  //typedef rclcpp_action::Client<move_base_msgs::action::MoveBaseAction> MoveBaseClient;

class RechargeController : public rclcpp::Node {  
  public:
    RechargeController();
    ~RechargeController();
    // void start();
    // void stop();

  private:
    // 机器人当前位置和回充点的坐标
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double recharge_x_;
    double recharge_y_;
    double recharge_yaw_;

    // 开始后退回充坐标点
    geometry_msgs::msg::PoseStamped back_point_;

    // 开始后退回充坐标点 与 充电点的距离
    double recharge_dist_;

    // 回充失败重试次数
    int recharge_repeat_;

    // // 后退时遇到障碍物累积时长计数
    // int time_obs_taltol_;

    // int charge_time_out_; // dxscpp
    // // 前进脱离充电桩时遇到障碍物累积时长计数
    // int front_time_obs_taltol_;

    // // 充电反馈成功后仍然后退时间
    // int time_chage_done_taltol_;

    // 控制频率
    int control_frequency_;

    // // 回充超时时间
    // double time_out_;

    // 角度误差
    double yaw_delta_;

    // 距离误差
    double dist_delta_;

    // // 底盘角度反馈
    // double angular_fb_;

    // 控制指令发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub_;

    // // 回充是否成功标志
    // bool is_charge_done_;

    // // 充电失败
    // bool is_charge_failed_;

    // bms反馈充电成功标志
    bool bms_charge_done_;

    // bool flag;
    // //bool static reset_last_time ;//dxscpp
    // //bool reset_last_time;

    // 回充点坐标更新回调函数
    bool rechargeCallback(
        const std::shared_ptr<yhs_can_interfaces::srv::Recharge::Request> request,
        const std::shared_ptr<yhs_can_interfaces::srv::Recharge::Response> response);
    bool disRechargeCallback(
        const std::shared_ptr<yhs_can_interfaces::srv::DisRecharge::Request> request,
        const std::shared_ptr<yhs_can_interfaces::srv::DisRecharge::Response> response);
 
    // // 机器人位置更新回调函数
    // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    // // 角度反馈回调函数
    // void angularCallback(const std_msgs::msg::Float32::SharedPtr angular_msg);

    // 控制指令发布函数
    void publishCmdVel(const double linear_vel, const double angular_vel);
    // void publishCmdVel(const double linear_vel, const double angular_vel);
    // void publishCtrlCmd(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle);

    // 计算最短路径
    inline double distanceToLine(Vector2d A, Vector2d dir, Vector2d B);

    // 回充控制函数
    void rechargeControl();

    // bool execute();

    // 导航到回充后退点
    bool goToBackPoint();

    // // 复位
    // void reset();

    // bms反馈
    // //void chassisInfoCallback(const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr chassis_info_msg); // dxs
    void chassisInfoCallback(const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr chassis_info_msg);

    // 线程相关的变量和函数
    // std::thread thread_;
    std::mutex mutex_;
    // std::condition_variable cond_var_;
    // bool continue_;
    // bool is_running_;
    // void threadFunction();

    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // //tf2_ros::Buffer tf_buffer_;
    // //tf2_ros::TransformListener tf_listener_;

    // // ROS节点句柄
    // //rclcpp::NodeHandle nh_;

    // // ROS订阅器
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // //rclcpp::Subscription<yhs_can_interfaces::msg::Recharge>::SharedPtr recharge_sub_;
    rclcpp::Subscription<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr bms_flag_sub_;

    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angular_fb_sub_;

    // //rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub_;

    // // Service
    // //rclcpp::Client<yhs_can_interfaces::srv::PathIsValid>::SharedPtr client_;
    // //rclcpp::Service<yhs_can_interfaces::srv::Recharge>::SharedPtr recharge_service_;
    // //rclcpp::Service<yhs_can_interfaces::srv::DisRecharge>::SharedPtr dis_recharge_service_;
    rclcpp::Service<yhs_can_interfaces::srv::Recharge>::SharedPtr recharge_service_;
    rclcpp::Service<yhs_can_interfaces::srv::DisRecharge>::SharedPtr dis_recharge_service_;

    // Action
    // //rclcpp_action::Client<move_base_msgs::action::MoveBase>::SharedPtr move_base_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  };
 // namespace recharge_controller_ns

#endif // RECHARGE_CONTROLLER_H