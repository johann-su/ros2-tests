#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

class SimplePublisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    int counter = 0;

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(counter++);
        this->publisher->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }


public:
    SimplePublisher() : Node("simple_publisher")
    {
        publisher = this->create_publisher<std_msgs::msg::String>("my_cool_topic", 10);
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimplePublisher::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();

  return 0;
}