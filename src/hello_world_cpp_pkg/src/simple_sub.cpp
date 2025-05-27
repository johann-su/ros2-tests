#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

class SimpleSubscriber : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    }

public:
    SimpleSubscriber() : Node("simple_subscriber")
    {
        subscription = this->create_subscription<std_msgs::msg::String>(
            "my_cool_topic", 10,
            std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}