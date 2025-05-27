#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hello_world_node");
    RCLCPP_INFO(node->get_logger(), "Hello, world! This is a simple ROS 2 node in C++.");
    rclcpp::shutdown();
    return 0;
}