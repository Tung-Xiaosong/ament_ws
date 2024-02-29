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

#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
//step 2 创建Node类的智能指针
rclcpp::Node::SharedPtr g_node = nullptr; //mark 初始时并没有指向任何对象。在全局下创建Node的指针，可以在全局中使用get_logger()

//step 6 创建服务端回调函数
void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  //step 1 初始化节点
  rclcpp::init(argc, argv);
  //step 3 指针指向"minimal_service"节点
  g_node = rclcpp::Node::make_shared("minimal_service");
  //step 4 创建server服务端
  auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
