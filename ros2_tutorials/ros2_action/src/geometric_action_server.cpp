#include "rclcpp/rclcpp.hpp"
#include "ros2_tutorials_interfaces/action/geometric_sequence.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GeometricActionServer : public rclcpp::Node
{
public:
using Geometric = ros2_tutorials_interfaces::action::GeometricSequence;
using GoalHandleGeometric = rclcpp_action::ServerGoalHandle<Geometric>;

explicit GeometricActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("geometric_action_server", options)
{
    using namespace std::placeholders;

    //step 2
    this->action_server_ = rclcpp_action::create_server<Geometric>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "geometric_sequence",
        std::bind(&GeometricActionServer::handle_goal, this, _1, _2),
        std::bind(&GeometricActionServer::handle_cancel, this, _1),
        std::bind(&GeometricActionServer::handle_accepted, this, _1));
}

private:
    //step 1
    rclcpp_action::Server<Geometric>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Geometric::Goal> goal)
    {

    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGeometric> goal_handle)
    {

    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleGeometric> goal_handle)
    {
        // RCLCPP_INFO(this->get_logger(), "Executing Goal");
        // rclcpp::Rate loop_rate(1);

    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeometricActionServer>());
    rclcpp::shutdown();
    return 0;
}