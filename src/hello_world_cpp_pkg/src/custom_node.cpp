#include "rclcpp/rclcpp.hpp"

class CustomNode : public rclcpp::Node
{
private:
    int counter = 0;
    rclcpp::TimerBase::SharedPtr timer;

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Time is ticking: %d", counter);
        counter++;
    }

public:
    CustomNode() : Node("custom_node")
    {
        RCLCPP_INFO(this->get_logger(), "CustomNode has been created.");

        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CustomNode::timer_callback, this));
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CustomNode>();
    RCLCPP_INFO(node->get_logger(), "Hello from CustomNode! This is a custom ROS 2 node in C++.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}