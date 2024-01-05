#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class buyerNodeClass : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr novel_sub;//声明订阅者

        void novelCallback(const std_msgs::msg::String::ConstPtr msg)//订阅者回调函数
        {
            RCLCPP_INFO(this->get_logger(), "I'm reading '%s'", msg->data.c_str());
        }
    public:
        buyerNodeClass(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "This is %s", name.c_str());
            novel_sub = this->create_subscription<std_msgs::msg::String>("novel", 10, std::bind(&buyerNodeClass::novelCallback, this, _1));//创建订阅者,订阅小说内容
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<buyerNodeClass>("buyer_node"));
    rclcpp::shutdown();
    return 0;
}
