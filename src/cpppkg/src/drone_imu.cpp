#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class DroneImu : public rclcpp::Node
{
public:
    DroneImu() : Node("drone_imu"), robot_name_("drone")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&DroneImu::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "drone imu has been started.");
    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the drone imu");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneImu>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
