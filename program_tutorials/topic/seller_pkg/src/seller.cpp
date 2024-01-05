#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

using std::placeholders::_1;

class sellerNodeClass : public rclcpp::Node
{
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr novel_pub;//声明发布者
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr money_sub;//声明订阅者
        rclcpp::TimerBase::SharedPtr timer;//声明定时器
        int count;//计数器
        int deposit;//存款

    public:
        sellerNodeClass(std::string name) : Node(name), count(0)/*计数器初始化*/, deposit(0)
        {
            RCLCPP_INFO(this->get_logger(), "This is %s", name.c_str());
            novel_pub = this->create_publisher<std_msgs::msg::String>("novel", 10);//创建发布者,发布小说内容
            timer = this->create_wall_timer(std::chrono::duration<double>(0.5), std::bind(&sellerNodeClass::publishNovel, this));//定时发布小说内容

            money_sub = this->create_subscription<std_msgs::msg::UInt16>("money", 10, std::bind(&sellerNodeClass::moneyCallback, this, _1));//创建订阅者,订阅稿费
        }

        void publishNovel()//发布小说内容
        {
            auto novel_content = std_msgs::msg::String();
            novel_content.data = "This is the novel content with chapter " + std::to_string(count);
            novel_pub->publish(novel_content);
            RCLCPP_INFO(this->get_logger(), "Publishing novel: '%s'", novel_content.data.c_str());
            count++;
        }

        void moneyCallback(const std_msgs::msg::UInt16::ConstPtr msg)//订阅稿费
        {
            deposit = deposit + msg->data;
            RCLCPP_INFO(this->get_logger(), "The remaining deposit is %d", deposit);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sellerNodeClass>("seller_node"));
    rclcpp::shutdown();
    return 0;
}

/*
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class WriterNode : public rclcpp::Node {
public:
    WriterNode(std::string name) : Node(name), i(0) {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s，我是一名作家！", name.c_str());

        pub_novel = this->create_publisher<std_msgs::msg::String>("sexy_girl", 10);

        // 创建定时器，每5秒写一章节话
        timer = this->create_wall_timer(std::chrono::seconds(5), std::bind(&WriterNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_novel;
    rclcpp::TimerBase::SharedPtr timer;
    int i; // 章节编号计数器

    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "第" + std::to_string(i) + "回：潋滟湖 " + std::to_string(i) + " 次偶遇胡艳娘";
        pub_novel->publish(msg);
        RCLCPP_INFO(this->get_logger(), "李四: 我发布了艳娘传奇：'%s'", msg.data.c_str());
        i++; // 章节编号自增
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WriterNode>("writer_node"));
    rclcpp::shutdown();
    return 0;
}

*/