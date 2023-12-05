#pragma once

#include <rclcpp/rclcpp.hpp>

#include <uav_msgs/msg/control_input.hpp>

class WinController
{
public:
    WinController(rclcpp::Node::SharedPtr node);
    void process();
private:

    rclcpp::Publisher<uav_msgs::msg::ControlInput>::SharedPtr m_pub_control_input;
};