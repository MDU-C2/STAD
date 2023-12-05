#pragma once

#include <rclcpp/rclcpp.hpp>

#include <uav_msgs/msg/kinematics.hpp>

class WinKinematics
{
public:
    WinKinematics(rclcpp::Node::SharedPtr node);
    void process();
private:
	rclcpp::Publisher<uav_msgs::msg::Kinematics>::SharedPtr m_pub_kinematics;
	rclcpp::Subscription<uav_msgs::msg::Kinematics>::SharedPtr m_sub_kinematics;
};