// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char * argv[])
{
  //step 1 初始化节点
  rclcpp::init(argc, argv);
  //step 2 创建指向节点的指针
  auto node = rclcpp::Node::make_shared("minimal_client");
  //step 3 创建客户端
  auto client = node->create_client<AddTwoInts>("add_two_ints");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  //step 4 创建request指针
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;
  //step 5 客户端发送请求
  //mark async_send_request 是一个在 ROS 2 的客户端（Client）中用于异步发送请求的方法。
  //* 这个方法允许客户端发送请求并立即返回一个 Future 对象, 也就是这里的result_future，该对象可以用于在后台异步地接收服务端（Server）发送回来的响应。
  //* 使用异步发送请求的方式可以使得客户端在等待响应的过程中不被阻塞，从而允许客户端在发送请求的同时继续执行其他操作。
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=//阻塞当前线程，直到给定的 Future 对象 result_future 完成
    rclcpp::FutureReturnCode::SUCCESS)//枚举值
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    client->remove_pending_request(result_future);//用于取消正在等待的请求
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(
    node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,//PRId64是一个宏，用于格式化输出int64_t类型的整数。
    request->a, request->b, result->sum);
  rclcpp::shutdown();
  return 0;
}
