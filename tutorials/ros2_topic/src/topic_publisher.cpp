#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
    //初始化节点
    rclcpp::init(argc, argv);
    std::cout<<"Publisher initialized !"<<std::endl;
    //创建节点对象
    auto node = std::make_shared<rclcpp::Node>("topic_publisher_node");
    //创建发布者
    auto publisher = node->create_publisher<std_msgs::msg::String>("/ros2_topic", 10);

    rclcpp::WallRate loop_rate(1);
    while(rclcpp::ok())
    {
        std_msgs::msg::String message;
        message.data = "hello world";
        publisher->publish(message);
        loop_rate.sleep();
    }
    //关闭节点
    rclcpp::shutdown();
    return 0;
}