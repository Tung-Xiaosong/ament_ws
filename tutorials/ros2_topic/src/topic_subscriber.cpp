#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void msgCallback(const std_msgs::msg::String & msg)
{
    std::cout<< "Subscriber heard : " << msg.data <<std::endl;
}
int main(int argc, char **argv)
{
    //初始化节点
    rclcpp::init(argc, argv);
    std::cout<<"Subscriber initialized !"<<std::endl;
    //创建节点对象
    auto node = std::make_shared<rclcpp::Node>("topic_subscriber_node");
    //创建订阅者
    auto subscriber = node->create_subscription<std_msgs::msg::String>(
        "/ros2_topic", 
        1,
        msgCallback);
    rclcpp::spin(node);
    //关闭节点
    rclcpp::shutdown();
    return 0;
}