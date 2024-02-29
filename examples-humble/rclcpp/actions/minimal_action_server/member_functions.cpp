// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <thread>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"  //mark

class MinimalActionServer : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  //step 4 创建服务端目标处理句柄 ,在Action服务器端，当接收到一个action目标时，会创建一个ServerGoalHandle对象来表示这个目标的处理状态
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>; 

  explicit MinimalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())//c++ explicit 声明构造函数为显式构造函数
  : Node("minimal_action_server", options)
  {
    using namespace std::placeholders;//mark 将std::placeholders命名空间中的所有内容引入到当前的命名空间中，这样在使用占位符时就不需要写std::placeholders::前缀了。
                                      //std::placeholders中定义了一些占位符，例如_1、_2等，通常用于绑定参数到函数对象中，比如在std::bind中使用

    //step 5 创建动作服务器
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this->get_node_base_interface(),  //获取节点的一些接口
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&MinimalActionServer::handle_goal, this, _1, _2),//处理目标(goal)的回调函数handle_goal
      std::bind(&MinimalActionServer::handle_cancel, this, _1),//处理取消(cancel)的回调函数handle_cancel
      std::bind(&MinimalActionServer::handle_accepted, this, _1));//处理接受(accept)的回调函数handle_accepted
  }

private:
  //step 3 创建动作服务器指针对象
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, //表示Action目标的唯一标识符。每个Action目标都有一个唯一的UUID。
    std::shared_ptr<const Fibonacci::Goal> goal)  //表示接收到的Action目标。
  {
    RCLCPP_INFO(this->get_logger(), "收到目标: %d", goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;//表示拒绝接受Action目标
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;//表示接受并执行Action目标
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消目标的请求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread这需要快速返回以避免阻塞执行器，因此启动一个新线程
    std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "执行目标");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    //step 6 创建feedback指针
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    //step 7 创建result指针
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "目标取消");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      //step 8 Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "发布反馈");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "目标成功");
    }
  }

};  // class MinimalActionServer

int main(int argc, char ** argv)
{
  //step 1 初始化节点
  rclcpp::init(argc, argv);
  //step 2 创建节点指针对象
  auto action_server = std::make_shared<MinimalActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
