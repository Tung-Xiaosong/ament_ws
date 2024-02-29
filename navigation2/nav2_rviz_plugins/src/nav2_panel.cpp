// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_rviz_plugins/nav2_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// Define global GoalPoseUpdater so that the nav2 GoalTool plugin can access to update goal pose
GoalPoseUpdater GoalUpdater;

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent),
  server_timeout_(100),
  loop_(false),
  recording_(false),
  goal_status_(0)
{
  // Create the control button and its tooltip

  start_reset_button_ = new QPushButton;
  pause_resume_button_ = new QPushButton;
  navigation_mode_button_ = new QPushButton;
  start_record_path_button_ = new QPushButton;
  load_record_path_button_ = new QPushButton;
  navigation_status_indicator_ = new QLabel;
  localization_status_indicator_ = new QLabel;
  navigation_goal_status_indicator_ = new QLabel;
  navigation_feedback_indicator_ = new QLabel;

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "配置并激活所有nav2生命周期节点";
  const char * shutdown_msg = "注销并清除所有nav2生命周期节点";
  const char * cancel_msg = "取消导航";
  const char * pause_msg = "注销所有nav2生命周期节点";
  const char * resume_msg = "激活所有nav2生命周期节点";
  const char * single_goal_msg = "切换多点或者路点导航模式";
  const char * waypoint_goal_msg = "开始多点导航";
  const char * nft_goal_msg = "开始路点导航";
  const char * cancel_waypoint_msg = "取消多点导航模式";

  const QString navigation_active("<table><tr><td width=100><b>导航初始化:</b></td>"
    "<td><font color=green>就绪</color></td></tr></table>");
  const QString navigation_inactive("<table><tr><td width=100><b>导航初始化:</b></td>"
    "<td>未就绪</td></tr></table>");
  const QString navigation_unknown("<table><tr><td width=100><b>导航初始化:</b></td>"
    "<td>未知</td></tr></table>");
  const QString localization_active("<table><tr><td width=100><b>定位初始化:</b></td>"
    "<td><font color=green>就绪</color></td></tr></table>");
  const QString localization_inactive("<table><tr><td width=100><b>定位初始化:</b></td>"
    "<td>未就绪</td></tr></table>");
  const QString localization_unknown("<table><tr><td width=100><b>定位初始化:</b></td>"
    "<td>未知</td></tr></table>");

  navigation_status_indicator_->setText(navigation_unknown);
  localization_status_indicator_->setText(localization_unknown);
  navigation_goal_status_indicator_->setText(getGoalStatusLabel());
  navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
  navigation_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  localization_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  navigation_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  navigation_feedback_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  pre_initial_ = new QState();
  pre_initial_->setObjectName("pre_initial");
  pre_initial_->assignProperty(start_reset_button_, "text", "开始");
  pre_initial_->assignProperty(start_reset_button_, "enabled", false);

  pre_initial_->assignProperty(pause_resume_button_, "text", "暂停");
  pre_initial_->assignProperty(pause_resume_button_, "enabled", false);

  pre_initial_->assignProperty(
    navigation_mode_button_, "text",
    "多点/路点模式");
  pre_initial_->assignProperty(navigation_mode_button_, "enabled", false);

  //开始录制路径按钮
  pre_initial_->assignProperty(start_record_path_button_, "text", "开始录制路径");
  pre_initial_->assignProperty(start_record_path_button_, "enabled", false);  

  //开始载入录制好的路径按钮
  pre_initial_->assignProperty(load_record_path_button_, "text", "载入路径");
  pre_initial_->assignProperty(load_record_path_button_, "enabled", false);  

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_reset_button_, "text", "开始");
  initial_->assignProperty(start_reset_button_, "toolTip", startup_msg);
  initial_->assignProperty(start_reset_button_, "enabled", true);

  initial_->assignProperty(pause_resume_button_, "text", "暂停");
  initial_->assignProperty(pause_resume_button_, "enabled", false);

  initial_->assignProperty(navigation_mode_button_, "text", "多点/路点模式");
  initial_->assignProperty(navigation_mode_button_, "enabled", false);

  // State entered when navigate_to_pose action is not active
  idle_ = new QState();
  idle_->setObjectName("idle");
  idle_->assignProperty(start_reset_button_, "text", "复位");
  idle_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);
  idle_->assignProperty(start_reset_button_, "enabled", true);

  idle_->assignProperty(pause_resume_button_, "text", "暂停");
  idle_->assignProperty(pause_resume_button_, "enabled", true);
  idle_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

  idle_->assignProperty(navigation_mode_button_, "text", "多点/路点模式");
  idle_->assignProperty(navigation_mode_button_, "enabled", true);
  idle_->assignProperty(navigation_mode_button_, "toolTip", single_goal_msg);

  //记录路径
  idle_->assignProperty(start_record_path_button_, "text", "开始记录路径");
  idle_->assignProperty(start_record_path_button_, "enabled", true);
  idle_->assignProperty(start_record_path_button_, "toolTip", "点击开始记录路径");

  //载入路径
  idle_->assignProperty(load_record_path_button_, "text", "载入路径并开始导航");
  idle_->assignProperty(load_record_path_button_, "enabled", true);
  idle_->assignProperty(load_record_path_button_, "toolTip", "载入录制好的路径后发布，并开始导航");


  // State entered when navigate_to_pose action is not active
  accumulating_ = new QState();
  accumulating_->setObjectName("accumulating");
  accumulating_->assignProperty(start_reset_button_, "text", "返回上级");
  accumulating_->assignProperty(start_reset_button_, "toolTip", cancel_waypoint_msg);
  accumulating_->assignProperty(start_reset_button_, "enabled", true);

  accumulating_->assignProperty(pause_resume_button_, "text", "开始路点导航");
  accumulating_->assignProperty(pause_resume_button_, "enabled", true);
  accumulating_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

  accumulating_->assignProperty(navigation_mode_button_, "text", "开始多点导航");
  accumulating_->assignProperty(navigation_mode_button_, "enabled", true);
  accumulating_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

  accumulating_->assignProperty(load_record_path_button_, "enabled", false);
  accumulating_->assignProperty(start_record_path_button_, "enabled", false);

  accumulated_wp_ = new QState();
  accumulated_wp_->setObjectName("accumulated_wp");
  accumulated_wp_->assignProperty(start_reset_button_, "text", "取消");
  accumulated_wp_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
  accumulated_wp_->assignProperty(start_reset_button_, "enabled", true);

  accumulated_wp_->assignProperty(pause_resume_button_, "text", "开始路点导航");
  accumulated_wp_->assignProperty(pause_resume_button_, "enabled", false);
  accumulated_wp_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

  accumulated_wp_->assignProperty(navigation_mode_button_, "text", "开始多点导航");
  accumulated_wp_->assignProperty(navigation_mode_button_, "enabled", false);
  accumulated_wp_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

  accumulated_nav_through_poses_ = new QState();
  accumulated_nav_through_poses_->setObjectName("accumulated_nav_through_poses");
  accumulated_nav_through_poses_->assignProperty(start_reset_button_, "text", "取消");
  accumulated_nav_through_poses_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
  accumulated_nav_through_poses_->assignProperty(start_reset_button_, "enabled", true);

  accumulated_nav_through_poses_->assignProperty(
    pause_resume_button_, "text",
    "开始路点导航");
  accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "enabled", false);
  accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

  accumulated_nav_through_poses_->assignProperty(
    navigation_mode_button_, "text",
    "开始多点导航");
  accumulated_nav_through_poses_->assignProperty(navigation_mode_button_, "enabled", false);
  accumulated_nav_through_poses_->assignProperty(
    navigation_mode_button_, "toolTip",
    waypoint_goal_msg);

  //正在录制路径状态
  recording_path_ = new QState();
  recording_path_->setObjectName("recording_path");
  recording_path_->assignProperty(start_record_path_button_, "text", "停止并保存路径");
  recording_path_->assignProperty(start_record_path_button_, "toolTip", "点击停止并保存路径");
  recording_path_->assignProperty(load_record_path_button_, "enabled", false);
  recording_path_->assignProperty(start_reset_button_, "enabled", false);
  recording_path_->assignProperty(pause_resume_button_, "enabled", false);
  recording_path_->assignProperty(navigation_mode_button_, "enabled", false);

  //录制路径
  go_to_record_path_init_pose_ = new QState();
  go_to_record_path_init_pose_->setObjectName("accumulated_follow_record_path");
  go_to_record_path_init_pose_->assignProperty(start_reset_button_, "text", "取消");
  go_to_record_path_init_pose_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
  go_to_record_path_init_pose_->assignProperty(start_reset_button_, "enabled", true);
  go_to_record_path_init_pose_->assignProperty(load_record_path_button_, "enabled", false);
  go_to_record_path_init_pose_->assignProperty(pause_resume_button_, "enabled", false);
  go_to_record_path_init_pose_->assignProperty(navigation_mode_button_, "enabled", false);
  go_to_record_path_init_pose_->assignProperty(start_record_path_button_, "enabled", false);

  accumulated_follow_record_path_ = new QState();
  accumulated_follow_record_path_->setObjectName("accumulated_follow_record_path");
  accumulated_follow_record_path_->assignProperty(start_reset_button_, "text", "取消");
  accumulated_follow_record_path_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
  accumulated_follow_record_path_->assignProperty(start_reset_button_, "enabled", true);
  accumulated_follow_record_path_->assignProperty(load_record_path_button_, "enabled", false);
  accumulated_follow_record_path_->assignProperty(pause_resume_button_, "enabled", false);
  accumulated_follow_record_path_->assignProperty(navigation_mode_button_, "enabled", false);
  accumulated_follow_record_path_->assignProperty(start_record_path_button_, "enabled", false);

  // State entered to cancel the navigate_to_pose action
  canceled_ = new QState();
  canceled_->setObjectName("canceled");

  // State entered to reset the nav2 lifecycle nodes
  reset_ = new QState();
  reset_->setObjectName("reset");

  // State entered while the navigate_to_pose action is active
  running_ = new QState();
  running_->setObjectName("running");
  running_->assignProperty(start_reset_button_, "text", "取消");
  running_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

  running_->assignProperty(pause_resume_button_, "text", "暂停");
  running_->assignProperty(pause_resume_button_, "enabled", false);

  running_->assignProperty(navigation_mode_button_, "text", "多点导航");
  running_->assignProperty(navigation_mode_button_, "enabled", false);

  //录制路径
  running_->assignProperty(start_record_path_button_, "enabled", false);
  running_->assignProperty(load_record_path_button_, "enabled", false);

  // State entered when pause is requested
  paused_ = new QState();
  paused_->setObjectName("pausing");
  paused_->assignProperty(start_reset_button_, "text", "复位");
  paused_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);

  paused_->assignProperty(pause_resume_button_, "text", "恢复");
  paused_->assignProperty(pause_resume_button_, "toolTip", resume_msg);
  paused_->assignProperty(pause_resume_button_, "enabled", true);

  paused_->assignProperty(navigation_mode_button_, "text", "开始导航");
  paused_->assignProperty(navigation_mode_button_, "toolTip", resume_msg);
  paused_->assignProperty(navigation_mode_button_, "enabled", true);

  // State entered to resume the nav2 lifecycle nodes
  resumed_ = new QState();
  resumed_->setObjectName("resuming");

  QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
  QObject::connect(canceled_, SIGNAL(exited()), this, SLOT(onCancel()));
  QObject::connect(reset_, SIGNAL(exited()), this, SLOT(onShutdown()));
  QObject::connect(paused_, SIGNAL(entered()), this, SLOT(onPause()));
  QObject::connect(resumed_, SIGNAL(exited()), this, SLOT(onResume()));
  QObject::connect(accumulating_, SIGNAL(entered()), this, SLOT(onAccumulating()));
  QObject::connect(accumulated_wp_, SIGNAL(entered()), this, SLOT(onAccumulatedWp()));
  QObject::connect(
    accumulated_nav_through_poses_, SIGNAL(entered()), this,
    SLOT(onAccumulatedNTP()));

  //开始记录按钮槽函数
  last_recorded_pose_.pose.position.x = 0.0;
  last_recorded_pose_.pose.position.y = 0.0;
  QObject::connect(start_record_path_button_, SIGNAL(clicked()), this, SLOT(onRecordButtonClicked()));
  idle_->addTransition(start_record_path_button_, SIGNAL(clicked()), recording_path_);
  recording_path_->addTransition(start_record_path_button_, SIGNAL(clicked()), idle_);

  //开始载入路径按钮
  idle_->addTransition(load_record_path_button_, SIGNAL(clicked()), go_to_record_path_init_pose_);
  QObject::connect(go_to_record_path_init_pose_, SIGNAL(entered()), this, SLOT(onLoadButtonClicked()));
  QObject::connect(accumulated_follow_record_path_, SIGNAL(entered()), this, SLOT(startFollowRecordPath()));
  accumulated_follow_record_path_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  go_to_record_path_init_pose_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
//  QObject::connect(load_record_path_button_, SIGNAL(clicked()), this, SLOT(onLoadButtonClicked()));

  // Start/Reset button click transitions
  initial_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  idle_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  running_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  paused_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  idle_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulating_);
  accumulating_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulated_wp_);
  accumulating_->addTransition(
    pause_resume_button_, SIGNAL(
      clicked()), accumulated_nav_through_poses_);
  accumulating_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  accumulated_wp_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  accumulated_nav_through_poses_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);

  // Internal state transitions
  canceled_->addTransition(canceled_, SIGNAL(entered()), idle_);
  reset_->addTransition(reset_, SIGNAL(entered()), initial_);
  resumed_->addTransition(resumed_, SIGNAL(entered()), idle_);

  // Pause/Resume button click transitions
  idle_->addTransition(pause_resume_button_, SIGNAL(clicked()), paused_);
  paused_->addTransition(pause_resume_button_, SIGNAL(clicked()), resumed_);

  // ROSAction Transitions: So when actions are updated remotely (failing, succeeding, etc)
  // the state of the application will also update. This means that if in the processing
  // states and then goes inactive, move back to the idle state. Vise versa as well.
  
  //获取导航状态后进入running_
  ROSActionQTransition * idleTransition = new ROSActionQTransition(QActionState::INACTIVE);
  idleTransition->setTargetState(running_);
  idle_->addTransition(idleTransition);

  ROSActionQTransition * runningTransition = new ROSActionQTransition(QActionState::ACTIVE);
  runningTransition->setTargetState(idle_);
  running_->addTransition(runningTransition);

  //导航结束进入idle_状态
  ROSActionQTransition * accumulatedFollowRecordPathTransition = new ROSActionQTransition(QActionState::ACTIVE);
  accumulatedFollowRecordPathTransition->setTargetState(idle_);
  accumulated_follow_record_path_->addTransition(accumulatedFollowRecordPathTransition);

  //到达录制路径起始点，进入accumulated_follow_record_path_状态
  ROSActionQTransition * goToFollowRecordPathTransition = new ROSActionQTransition(QActionState::ACTIVE);
  goToFollowRecordPathTransition->setTargetState(accumulated_follow_record_path_);
  go_to_record_path_init_pose_->addTransition(goToFollowRecordPathTransition);

  // ROSActionQTransition * idleAccumulatedWpTransition =
  //   new ROSActionQTransition(QActionState::INACTIVE);
  // idleAccumulatedWpTransition->setTargetState(accumulated_wp_);
  // idle_->addTransition(idleAccumulatedWpTransition);

  //导航结束进入idle_状态
  ROSActionQTransition * accumulatedWpTransition = new ROSActionQTransition(QActionState::ACTIVE);
  accumulatedWpTransition->setTargetState(idle_);
  accumulated_wp_->addTransition(accumulatedWpTransition);

  // ROSActionQTransition * idleAccumulatedNTPTransition =
  //   new ROSActionQTransition(QActionState::INACTIVE);
  // idleAccumulatedNTPTransition->setTargetState(accumulated_nav_through_poses_);
  // idle_->addTransition(idleAccumulatedNTPTransition);

  //导航结束进入idle_状态
  ROSActionQTransition * accumulatedNTPTransition = new ROSActionQTransition(QActionState::ACTIVE);
  accumulatedNTPTransition->setTargetState(idle_);
  accumulated_nav_through_poses_->addTransition(accumulatedNTPTransition);

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  client_nav_ = std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_navigation", client_node_);
  client_loc_ = std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_localization", client_node_);
  initial_thread_ = new InitialThread(client_nav_, client_loc_);
  connect(initial_thread_, &InitialThread::finished, initial_thread_, &QObject::deleteLater);

  //导航就绪，进入idle_状态
  QSignalTransition * activeSignal = new QSignalTransition(
    initial_thread_,
    &InitialThread::navigationActive);
  activeSignal->setTargetState(idle_);
  pre_initial_->addTransition(activeSignal);

  QSignalTransition * inactiveSignal = new QSignalTransition(
    initial_thread_,
    &InitialThread::navigationInactive);
  inactiveSignal->setTargetState(initial_);
  pre_initial_->addTransition(inactiveSignal);

  QObject::connect(
    initial_thread_, &InitialThread::navigationActive,
    [this, navigation_active] {
      navigation_status_indicator_->setText(navigation_active);
    });
  QObject::connect(
    initial_thread_, &InitialThread::navigationInactive,
    [this, navigation_inactive] {
      navigation_status_indicator_->setText(navigation_inactive);
      navigation_goal_status_indicator_->setText(getGoalStatusLabel());
      navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
    });
  QObject::connect(
    initial_thread_, &InitialThread::localizationActive,
    [this, localization_active] {
      localization_status_indicator_->setText(localization_active);
    });
  QObject::connect(
    initial_thread_, &InitialThread::localizationInactive,
    [this, localization_inactive] {
      localization_status_indicator_->setText(localization_inactive);
    });

  state_machine_.addState(pre_initial_);
  state_machine_.addState(initial_);
  state_machine_.addState(idle_);
  state_machine_.addState(running_);
  state_machine_.addState(canceled_);
  state_machine_.addState(reset_);
  state_machine_.addState(paused_);
  state_machine_.addState(resumed_);
  state_machine_.addState(accumulating_);
  state_machine_.addState(accumulated_wp_);
  state_machine_.addState(accumulated_nav_through_poses_);
  state_machine_.addState(recording_path_);
  state_machine_.addState(go_to_record_path_init_pose_);
  state_machine_.addState(accumulated_follow_record_path_);

  state_machine_.setInitialState(pre_initial_);

  // delay starting initial thread until state machine has started or a race occurs
  QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(navigation_status_indicator_);
  main_layout->addWidget(localization_status_indicator_);
  main_layout->addWidget(navigation_goal_status_indicator_);
  main_layout->addWidget(navigation_feedback_indicator_);
  main_layout->addWidget(pause_resume_button_);
  main_layout->addWidget(start_reset_button_);
  main_layout->addWidget(navigation_mode_button_);

  QHBoxLayout *horizontalLayout = new QHBoxLayout();
  horizontalLayout->addWidget(start_record_path_button_);
  horizontalLayout->addWidget(load_record_path_button_);

  main_layout->addLayout(horizontalLayout);

  // 创建复位导航点按钮
  QPushButton *poseReset = new QPushButton("复位导航点");
  main_layout->addWidget(poseReset, 0, Qt::AlignLeft | Qt::AlignBottom);

  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);

  // 创建复选框
  QCheckBox *checkBox = new QCheckBox("多点导航循环");
  main_layout->addWidget(checkBox, 0, Qt::AlignLeft | Qt::AlignBottom);

  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);


  //复选框槽函数
  connect(checkBox, SIGNAL(stateChanged(int)), this, SLOT(onCheckBoxStateChanged(int)));

  //复位导航点按钮槽函数
  connect(poseReset, SIGNAL(clicked()), this, SLOT(onPoseResetButtonClicked()));

  navigation_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_,
    "navigate_to_pose");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    client_node_,
    "follow_waypoints");
  nav_through_poses_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    client_node_,
    "navigate_through_poses");

  record_path_action_client_ =
    rclcpp_action::create_client<yhs_can_interfaces::action::RecordPath>(
    client_node_,
    "navigate_follow_record_path");

  navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
  nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
  record_path_goal_ = yhs_can_interfaces::action::RecordPath::Goal();

  wp_navigation_markers_pub_ =
    client_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "waypoints",
    rclcpp::QoS(1).transient_local());

  //发布记录好的路径
  record_path_publisher_ =
    client_node_->create_publisher<nav_msgs::msg::Path>(
    "record_path_rviz",
    rclcpp::QoS(1).transient_local());

  //发布导航类型
  nav_type_publisher_ =
    client_node_->create_publisher<std_msgs::msg::UInt8>(
    "nav_type",
    rclcpp::QoS(1).transient_local());

  QObject::connect(
    &GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),                 // NOLINT
    this, SLOT(onNewGoal(double,double,double,QString)));  // NOLINT
}

Nav2Panel::~Nav2Panel()
{
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  record_path_file_ = ament_index_cpp::get_package_share_directory("nav2_rviz_plugins") + "/data/recorded_path.txt";

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // create action feedback subscribers
  navigation_feedback_sub_ =
    node->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
    "navigate_to_pose/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {
      navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel(msg->feedback));
    });
  nav_through_poses_feedback_sub_ =
    node->create_subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
    "navigate_through_poses/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg) {
      navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel(msg->feedback));
    });

  record_path_feedback_sub_ =
    node->create_subscription<yhs_can_interfaces::action::RecordPath::Impl::FeedbackMessage>(
    "navigate_follow_record_path/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const yhs_can_interfaces::action::RecordPath::Impl::FeedbackMessage::SharedPtr msg) {
      navigation_feedback_indicator_->setText(QString(
    std::string(
      "<table><tr><td width=150>剩余导航点:</td><td>" +
      std::to_string(1) +
      "</td></tr>" + toLabel(msg->feedback) + "</table>").c_str()));
    });

  // create action goal status subscribers
  navigation_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      navigation_goal_status_indicator_->setText(
        getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        goal_status_ = msg->status_list.back().status;
        navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel());
      }
    });
  nav_through_poses_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_through_poses/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      navigation_goal_status_indicator_->setText(
        getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
      }
    });

  record_path_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_follow_record_path/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      navigation_goal_status_indicator_->setText(
        getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
      }
    });
}

void
Nav2Panel::startThread()
{
  // start initial thread now that state machine is started
  initial_thread_->start();
}

void
Nav2Panel::onPause()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::pause,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::pause,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onResume()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::resume,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::resume,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onStartup()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::startup,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::startup,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onShutdown()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::reset,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::reset,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
  timer_.stop();
}

void
Nav2Panel::onCancel()
{
  QFuture<void> future =
    QtConcurrent::run(
    std::bind(
      &Nav2Panel::onCancelButtonPressed,
      this));
}

void
Nav2Panel::onNewGoal(double x, double y, double theta, QString frame)
{
  auto pose = geometry_msgs::msg::PoseStamped();

  pose.header.stamp = rclcpp::Clock().now();
  pose.header.frame_id = frame.toStdString();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientationAroundZAxis(theta);

  if (state_machine_.configuration().contains(accumulating_)) {
    acummulated_poses_.push_back(pose);
  } else {
    std::cout << "Start navigation" << std::endl;
    acummulated_poses_.clear();
    acummulated_poses_.push_back(pose);
    startNavigation(pose);
  }

  std_msgs::msg::UInt8 nav_type_msg;
  nav_type_msg.data = 0;
  nav_type_publisher_->publish(nav_type_msg);

  updateWpNavigationMarkers();
}

//载入路径槽函数
void 
Nav2Panel::onLoadButtonClicked() {
  // 读取a.txt文件
  std::ifstream file(record_path_file_);
  if (!file.is_open()) {
    std::cout <<  "打开记录路径失败!" << std::endl;
    return;
  }

  // 统计文件行数
  int line_count = std::count(std::istreambuf_iterator<char>(file),
                              std::istreambuf_iterator<char>(), '\n') + 1;

  // 如果行数不足10行，则返回false
  if (line_count <= 10) {
    std::cout <<  "路径数据较少，请重新记录!" << std::endl;
    return;
  }

  // 重新定位文件指针到文件开头
  file.clear();
  file.seekg(0, std::ios::beg);

  // 读取文件中的坐标和yaw，并发布路径消息
  record_path_msg_.header.frame_id = "map";
  record_path_msg_.header.stamp = rclcpp::Clock().now();
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double x, y, yaw;
    char comma;
    if (iss >> x >> comma >> y >> comma >> yaw) {
      // 创建PoseStamped消息
      geometry_msgs::msg::PoseStamped pose;

      pose.header.frame_id = "map";

      pose.pose.position.x = x;
      pose.pose.position.y = y;

      // 欧拉角转四元数
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, yaw);
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();

      // 将PoseStamped消息添加到路径中
      record_path_msg_.poses.push_back(pose);
    }
  }
  record_path_publisher_->publish(record_path_msg_);
  std::cout << "打开记录路径并且发布成功！" << std::endl;

  //先导航到路径起始点
  record_path_msg_.poses[0].header.stamp = rclcpp::Clock().now();
  record_path_msg_.poses[0].header.frame_id = "map";

  startNavigation(record_path_msg_.poses[0]);

  std_msgs::msg::UInt8 nav_type_msg;
  nav_type_msg.data = 0;
  nav_type_publisher_->publish(nav_type_msg);

//  startFollowRecordPath();
}

// 槽函数，处理复选框状态变化
void 
Nav2Panel::onCheckBoxStateChanged(int state) {
  // 将复选框的状态转换为bool类型，然后赋给loop_变量
  loop_ = (state == Qt::Checked);

  if(loop_) std::cout << "设置导航循环！" << std::endl;
  else std::cout << "取消导航循环！" << std::endl;
}

// 槽函数，处理复位导航点按钮
void 
Nav2Panel::onPoseResetButtonClicked() {

  if (!state_machine_.configuration().contains(accumulated_wp_) && !state_machine_.configuration().contains(accumulated_nav_through_poses_)) {
    acummulated_poses_.clear();
    updateWpNavigationMarkers();
    std::cout << "已清除所有导航点！" << std::endl;
  }
}

//录制路径槽函数
void
Nav2Panel::onRecordButtonClicked(){
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  recording_ = !recording_;
  if (recording_) {
//    start_record_path_button_->setText("停止记录路径");
    outfile_.open(record_path_file_, std::ios::out | std::ios::trunc);

    // 设置定时器
    record_path_timer_ = node->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Nav2Panel::updateRobotPose, this));
  } else {
//    start_record_path_button_->setText("开始记录路径");
    // Close the file
    outfile_.close();

    // 停止定时器
    if (record_path_timer_) {
        record_path_timer_->cancel();
    }
  }
}

void 
Nav2Panel::updateRobotPose() {
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  try {
    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "base_link",tf2::TimePointZero);

    double distance = std::sqrt(std::pow(transform_stamped.transform.translation.x - last_recorded_pose_.pose.position.x, 2) +
                                std::pow(transform_stamped.transform.translation.y - last_recorded_pose_.pose.position.y, 2));

    // 如果距离大于10cm，将机器人的坐标保存到文件
    if (distance > 0.1) {
      outfile_ << transform_stamped.transform.translation.x << "," << transform_stamped.transform.translation.y
                << "," << tf2::getYaw(transform_stamped.transform.rotation) << std::endl;

      // 更新上一次记录的位置
      last_recorded_pose_.pose.position.x = transform_stamped.transform.translation.x;
      last_recorded_pose_.pose.position.y = transform_stamped.transform.translation.y;
    }
  } catch (tf2::TransformException &ex) {
    // 处理异常（例如，坐标变换关系不存在）
    RCLCPP_ERROR(node->get_logger(), "Tf_listener Exception: %s", ex.what());
  }
 
}

void
Nav2Panel::onCancelButtonPressed()
{
  if (navigation_goal_handle_) {
    auto future_cancel = navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      navigation_goal_handle_.reset();
    }
  }

  if (waypoint_follower_goal_handle_) {
    auto future_cancel =
      waypoint_follower_action_client_->async_cancel_goal(waypoint_follower_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel waypoint follower");
    } else {
      waypoint_follower_goal_handle_.reset();
    }
  }

  if (nav_through_poses_goal_handle_) {
    auto future_cancel =
      nav_through_poses_action_client_->async_cancel_goal(nav_through_poses_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel nav through pose action");
    } else {
      nav_through_poses_goal_handle_.reset();
    }
  }

  if (record_path_goal_handle_) {
    auto future_cancel =
      record_path_action_client_->async_cancel_goal(record_path_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel follow record path action");
    } else {
      record_path_goal_handle_.reset();
    }
  }


  timer_.stop();
}

void
Nav2Panel::onAccumulatedWp()
{
  std::cout << "Start waypoint" << std::endl;
  startWaypointFollowing(acummulated_poses_);
//  acummulated_poses_.clear();

  std_msgs::msg::UInt8 nav_type_msg;
  nav_type_msg.data = 0;
  nav_type_publisher_->publish(nav_type_msg);
}

void
Nav2Panel::onAccumulatedNTP()
{
  std::cout << "Start navigate through poses" << std::endl;
  startNavThroughPoses(acummulated_poses_);
//  acummulated_poses_.clear();

  std_msgs::msg::UInt8 nav_type_msg;
  nav_type_msg.data = 1;
  nav_type_publisher_->publish(nav_type_msg);
}

void
Nav2Panel::onAccumulating()
{
//  acummulated_poses_.clear();
}

void
Nav2Panel::timerEvent(QTimerEvent * event)
{
  if (state_machine_.configuration().contains(accumulated_wp_)) {
    if (event->timerId() == timer_.timerId()) {
      if (!waypoint_follower_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = waypoint_follower_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {

        if(!loop_) {
          state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
          timer_.stop();
        }
        else {
          onAccumulatedWp();
        }
      }
    }
  } else if (state_machine_.configuration().contains(accumulated_nav_through_poses_)) {
    if (event->timerId() == timer_.timerId()) {
      if (!nav_through_poses_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = nav_through_poses_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        timer_.stop();
      }
    }
  } else if (state_machine_.configuration().contains(accumulated_follow_record_path_)) {
    if (event->timerId() == timer_.timerId()) {
      if (!record_path_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = record_path_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        timer_.stop();
      }
    }
  } else if (state_machine_.configuration().contains(go_to_record_path_init_pose_)) {
    if (event->timerId() == timer_.timerId()) {
      if (!navigation_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = navigation_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        timer_.stop();
      }
    }
  } 
  else {
    if (event->timerId() == timer_.timerId()) {
      if (!navigation_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = navigation_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        timer_.stop();
      }
    }
  }
}

void
Nav2Panel::startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      waypoint_follower_goal_handle_.reset();
    };

  auto future_goal_handle =
    waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}

void
Nav2Panel::startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return;
  }

  nav_through_poses_goal_.poses = poses;
  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateThroughPoses will be called using the BT Navigator's default behavior tree.");

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal_.poses.size());
  for (auto waypoint : nav_through_poses_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      nav_through_poses_goal_handle_.reset();
    };

  auto future_goal_handle =
    nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  nav_through_poses_goal_handle_ = future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}

void
Nav2Panel::startNavigation(geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready =
    navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal pose
  navigation_goal_.pose = pose;

  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      navigation_goal_handle_.reset();
    };

  auto future_goal_handle =
    navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  navigation_goal_handle_ = future_goal_handle.get();
  if (!navigation_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}


//开始录制路径客户端请求
void
Nav2Panel::startFollowRecordPath()
{
  //不能到达起始点
  if(goal_status_ != 4) {
    //进入idle_状态
    state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
    return;
  }

  auto is_action_server_ready =
    record_path_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_follow_record_path action server is not available."
      " Is the initial pose set?");
    return;
  }

  //存入路径
  record_path_goal_.path = record_path_msg_;

  record_path_msg_.poses.clear();

  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<yhs_can_interfaces::action::RecordPath>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      record_path_goal_handle_.reset();
    };

  auto future_goal_handle =
    record_path_action_client_->async_send_goal(record_path_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  record_path_goal_handle_ = future_goal_handle.get();
  if (!record_path_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }
  
  std_msgs::msg::UInt8 nav_type_msg;
  nav_type_msg.data = 1;
  nav_type_publisher_->publish(nav_type_msg);

  timer_.start(200, this);
}

void
Nav2Panel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
Nav2Panel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void
Nav2Panel::resetUniqueId()
{
  unique_id = 0;
}

int
Nav2Panel::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}

void
Nav2Panel::updateWpNavigationMarkers()
{
  resetUniqueId();

  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

  for (size_t i = 0; i < acummulated_poses_.size(); i++) {
    // Draw a green arrow at the waypoint pose
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header = acummulated_poses_[i].header;
    arrow_marker.id = getUniqueId();
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose = acummulated_poses_[i].pose;
    arrow_marker.scale.x = 1.2;
    arrow_marker.scale.y = 0.2;
    arrow_marker.scale.z = 0.02;
    arrow_marker.color.r = 0;
    arrow_marker.color.g = 255;
    arrow_marker.color.b = 0;
    arrow_marker.color.a = 1.0f;
    arrow_marker.lifetime = rclcpp::Duration(0s);
    arrow_marker.frame_locked = false;
    marker_array->markers.push_back(arrow_marker);

    // Draw a red circle at the waypoint pose
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.header = acummulated_poses_[i].header;
    circle_marker.id = getUniqueId();
    circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;
    circle_marker.pose = acummulated_poses_[i].pose;
    circle_marker.scale.x = 0.2;
    circle_marker.scale.y = 0.2;
    circle_marker.scale.z = 0.05;
    circle_marker.color.r = 255;
    circle_marker.color.g = 0;
    circle_marker.color.b = 0;
    circle_marker.color.a = 1.0f;
    circle_marker.lifetime = rclcpp::Duration(0s);
    circle_marker.frame_locked = false;
    marker_array->markers.push_back(circle_marker);

    // Draw the waypoint number
    visualization_msgs::msg::Marker marker_text;
    marker_text.header = acummulated_poses_[i].header;
    marker_text.id = getUniqueId();
    marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::msg::Marker::ADD;
    marker_text.pose = acummulated_poses_[i].pose;
    marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
    marker_text.scale.x = 0.2;
    marker_text.scale.y = 0.2;
    marker_text.scale.z = 0.2;
    marker_text.color.r = 0;
    marker_text.color.g = 255;
    marker_text.color.b = 0;
    marker_text.color.a = 1.0f;
    marker_text.lifetime = rclcpp::Duration(0s);
    marker_text.frame_locked = false;
    marker_text.text = "WP_" + std::to_string(i + 1);
    marker_array->markers.push_back(marker_text);
  }

  if (marker_array->markers.empty()) {
    visualization_msgs::msg::Marker clear_all_marker;
    clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(clear_all_marker);
  }

  wp_navigation_markers_pub_->publish(std::move(marker_array));
}

inline QString
Nav2Panel::getGoalStatusLabel(int8_t status)
{
  std::string status_str;
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      status_str = "<font color=green>执行</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      status_str = "<font color=green>到达</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      status_str = "<font color=orange>取消</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      status_str = "<font color=red>不能到达</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      status_str = "未知";
      break;

    default:
      status_str = "未就绪";
      break;
  }
  return QString(
    std::string(
      "<table><tr><td width=100><b>导航状态反馈:</b></td><td>" +
      status_str + "</td></tr></table>").c_str());
}

inline QString
Nav2Panel::getNavToPoseFeedbackLabel(nav2_msgs::action::NavigateToPose::Feedback msg)
{
  return QString(std::string("<table>" + toLabel(msg) + "</table>").c_str());
}

inline QString
Nav2Panel::getNavThroughPosesFeedbackLabel(nav2_msgs::action::NavigateThroughPoses::Feedback msg)
{
  return QString(
    std::string(
      "<table><tr><td width=150>剩余导航点:</td><td>" +
      std::to_string(msg.number_of_poses_remaining) +
      "</td></tr>" + toLabel(msg) + "</table>").c_str());
}

template<typename T>
inline std::string Nav2Panel::toLabel(T & msg)
{
  return std::string(
    "<tr><td width=150>ETA:</td><td>" +
    toString(rclcpp::Duration(msg.estimated_time_remaining).seconds(), 0) + " s"
    "</td></tr><tr><td width=150>剩余路径:</td><td>" +
    toString(msg.distance_remaining, 2) + " m"
    "</td></tr><tr><td width=150>花费时间:</td><td>" +
    toString(rclcpp::Duration(msg.navigation_time).seconds(), 0) + " s"
    "</td></tr><tr><td width=150>恢复次数:</td><td>" +
    std::to_string(msg.number_of_recoveries) +
    "</td></tr>");
}

inline std::string
Nav2Panel::toString(double val, int precision)
{
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << val;
  return out.str();
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)