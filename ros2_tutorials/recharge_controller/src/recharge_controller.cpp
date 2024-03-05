#include "recharge_controller/recharge_controller.hpp"

RechargeController::RechargeController() : Node("recharge_controller_node")
  , recharge_dist_(0.8)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //参数
  control_frequency_ = this->declare_parameter<int>("recharge_control_frequency", 33);
  recharge_repeat_ = this->declare_parameter<int>("recharge_repeat", 3);

  //topic
  //发布速度话题
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  //发布控制话题
  ctrl_cmd_pub_ = this->create_publisher<yhs_can_interfaces::msg::CtrlCmd>("ctrl_cmd", 1);
  
  //订阅bms信息
  bms_flag_sub_ = this->create_subscription<yhs_can_interfaces::msg::ChassisInfoFb>("chassis_info_fb", 1, \
  std::bind(&RechargeController::chassisInfoCallback, this, std::placeholders::_1));

  //service
  //创建回充服务端
  recharge_service_ = this->create_service<yhs_can_interfaces::srv::Recharge>(
          "recharge",
          std::bind(&RechargeController::rechargeCallback, this, std::placeholders::_1, std::placeholders::_2));

  //创建脱离充电桩服务端
  dis_recharge_service_ = this->create_service<yhs_can_interfaces::srv::DisRecharge>(
          "dis_recharge",
          std::bind(&RechargeController::disRechargeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

// bool RechargeController::rechargeCallback(const std::shared_ptr<rmw_request_id_t> request_header,
//                            const std::shared_ptr<yhs_can_interfaces::srv::Recharge::Request> request,
//                            const std::shared_ptr<yhs_can_interfaces::srv::Recharge::Response> response)
// {

// }

// done
bool RechargeController::goToBackPoint()
{
  // 创建“navigate_to_pose”action客户端
  navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>
  (
    this,
    "navigate_to_pose");

  // 等待连接服务器
  auto is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if(!is_action_server_ready)
  {
    RCLCPP_ERROR(this->get_logger(), "Waiting for the navigate_to_pose action server to come up !");
    //return 0; // test
  }

  // 发送导航点请求
  nav2_msgs::action::NavigateToPose::Goal navigation_goal = nav2_msgs::action::NavigateToPose::Goal();

  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = this->now();
  navigation_goal.pose.pose = back_point_.pose;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  auto future_goal_handle = navigation_action_client_->async_send_goal(navigation_goal, send_goal_options);
  std::chrono::milliseconds server_timeout(1000);

  if(rclcpp::spin_until_future_complete(shared_from_this(), future_goal_handle, server_timeout) == 
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Reach back point !");
    return true;
  }  

  RCLCPP_ERROR(this->get_logger(), "Failed to reach back point !");
  return false;
}

bool RechargeController::rechargeCallback(
  const std::shared_ptr<yhs_can_interfaces::srv::Recharge::Request> request,
  const std::shared_ptr<yhs_can_interfaces::srv::Recharge::Response> response)
{
  if(bms_charge_done_)
  {
    response->result = 1;
    return true;
  }
  // 更新回充点坐标
  recharge_x_ = request->recharge_goal.pose.position.x;
  recharge_y_ = request->recharge_goal.pose.position.y;
  tf2::Quaternion q(
    request->recharge_goal.pose.orientation.x,
    request->recharge_goal.pose.orientation.y,
    request->recharge_goal.pose.orientation.z,
    request->recharge_goal.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  recharge_yaw_ = yaw;

  back_point_.pose.orientation = request->recharge_goal.pose.orientation;
  back_point_.pose.position.x = recharge_x_ + std::cos(recharge_yaw_) * recharge_dist_;
  back_point_.pose.position.y = recharge_x_ + std::cos(recharge_yaw_) * recharge_dist_;
  back_point_.pose.position.z = 0;

  bool to_back_point = goToBackPoint();

  if(to_back_point)
  {
    publishCmdVel(0, 0);
    sleep(2);
    {
      std::lock_guard<std::mutex> lock(mutex_);
    }

    rclcpp::WallRate loop_rate(control_frequency_);
    //loop_rate.sleep(); 

    int repeat_times = 0;
    while (rclcpp::ok() && repeat_times <= recharge_repeat_)
    {
      // 执行回充
      rechargeControl();
    }
    
  }
}

bool RechargeController::disRechargeCallback(
  const std::shared_ptr<yhs_can_interfaces::srv::DisRecharge::Request> request,
  const std::shared_ptr<yhs_can_interfaces::srv::DisRecharge::Response> response)
{

}

void RechargeController::rechargeControl()
{
  // 使用互斥锁保护访问类成员变量的线程安全
  std::lock_guard<std::mutex> lock(mutex_);

  // 使用tf2_ros包中的Buffer和TransformListener类来获取坐标变换
  try
  {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    // 获取机器人当前坐标
    robot_x_ = transform.transform.translation.x;
    robot_y_ = transform.transform.translation.y;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

// 发布控制指令 // done
void RechargeController::publishCmdVel(const double linear_vel, const double angular_vel)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;
  cmd_vel_pub_->publish(cmd_vel);
}

void RechargeController::chassisInfoCallback(const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr chassis_info_msg)
{

}





RechargeController::~RechargeController()
{

}