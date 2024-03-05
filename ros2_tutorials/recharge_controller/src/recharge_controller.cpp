#include "recharge_controller/recharge_controller.hpp"

RechargeController::RechargeController() : Node("recharge_controller_node")
  , recharge_dist_(0.8)
  , bms_charge_done_(false)
  , yaw_delta_(0.05)
  , dist_delta_(0.005)
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
  back_point_.pose.position.y = recharge_x_ + std::sin(recharge_yaw_) * recharge_dist_;
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

  // 使用tf2_ros包中的Buffer和TransformListener类来获取坐标变换，获取base_link机器人当前坐标
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

  geometry_msgs::msg::PointStamped recharge_point;// 回充点
  recharge_point.header.frame_id = "map";
  recharge_point.point.x = recharge_x_;
  recharge_point.point.y = recharge_y_;
  recharge_point.point.z = 0.0;
  // 将回充点坐标从map下转换为base_link下
  geometry_msgs::msg::PointStamped recharge_point_to_base_link_point;
  try
  {
    recharge_point_to_base_link_point = tf_buffer_->transform(recharge_point, "base_link");
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
    return;
  }

  // 计算机器人当前位置到回充点的距离和方向
  double dx_to_recharge_point = robot_x_ - recharge_x_;
  double dy_to_recharge_point = robot_y_ - recharge_y_;
  double dist_to_recharge_point = std::sqrt(dx_to_recharge_point * dx_to_recharge_point + dy_to_recharge_point * dy_to_recharge_point);
  //double yaw_diff_to_recharge_point = tf2::getYaw(tf2::Quaternion(0, 0, sin((robot_yaw_ - recharge_yaw_) / 2), cos((robot_yaw_ - recharge_yaw_) / 2)));
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, (robot_yaw_ - recharge_yaw_) / 2);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  double yaw_diff_to_recharge_point = yaw;

  Eigen::Vector2d A(recharge_x_, recharge_y_);// 使用Vector2d类创建了A和B两个二维向量对象，分别表示充电桩的位置(recharge_x_, recharge_y_)和机器人的位置(robot_x_, robot_y_)
  double angle = recharge_yaw_;
  Eigen::Vector2d dir(cos(angle), sin(angle));// 将充电桩的朝向recharge_yaw_通过cos()和sin()函数分别计算出方向向量dir的x和y分量
  Eigen::Vector2d B(robot_x_, robot_y_);
  double shortest_dist = distanceToLine(A, dir, B);// 计算了机器人当前位置与充电桩之间的最短距离。（垂直于充电桩的最短距离）

  // --------------------
  // 计算机器人需要的线速度和角速度
  double linear_vel = -0.08;
  double angular_vel = 0.0;
  unsigned char gear = 7;
  double slipangle = 0.0;

  if(!bms_charge_done_)
  {
    if(fabs(yaw_diff_to_recharge_point) >= yaw_delta_ && fabs(shortest_dist) > 0.5)// 角度相差太大，原地旋转调整角度
    {
      yaw_delta_ = 0.05;
      angular_vel = yaw_diff_to_recharge_point > 0 ? -0.08 : 0.08;
      linear_vel = 0.0;
      gear = 6;
    }
    if(fabs(yaw_diff_to_recharge_point) < yaw_delta_)
    {
      yaw_delta_ = 0.1;
    }

    // 往左偏了
    if(recharge_point_to_base_link_point.point.y > 0 && fabs(shortest_dist) > dist_delta_)
    {
      linear_vel = -0.08; 
      angular_vel = 0.0;
      slipangle = -7;
      dist_delta_ = 0.005;
      gear = 7;
    }
  }
}

double RechargeController::distanceToLine(Vector2d A, Vector2d dir, Vector2d B)
{
  Vector2d AB = B - A;
  Vector2d u = dir.normalized();
  double AP = AB.dot(u);
  double dist = sqrt(AB.dot(AB) - AP * AP);
  return dist;
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