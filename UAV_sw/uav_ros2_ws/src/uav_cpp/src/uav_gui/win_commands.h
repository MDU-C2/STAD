#pragma once

#include <rclcpp/rclcpp.hpp>

// messages
#include <uav_msgs/msg/arm.hpp>
#include <uav_msgs/msg/arm_ack.hpp>
#include <uav_msgs/msg/disarm.hpp>
#include <uav_msgs/msg/disarm_ack.hpp>
#include <uav_msgs/msg/armed_heartbeat.hpp>
#include <uav_msgs/msg/control_mode.hpp>

class WinCommands
{
public:
    WinCommands(rclcpp::Node::SharedPtr node);
    void process();
private:

    // Publishers
    rclcpp::Publisher<uav_msgs::msg::Arm>::SharedPtr m_pub_arm;
	rclcpp::Publisher<uav_msgs::msg::ArmAck>::SharedPtr m_pub_arm_ack;
	rclcpp::Publisher<uav_msgs::msg::Disarm>::SharedPtr m_pub_disarm;
	rclcpp::Publisher<uav_msgs::msg::DisarmAck>::SharedPtr m_pub_disarm_ack;
	rclcpp::Publisher<uav_msgs::msg::ArmedHeartbeat>::SharedPtr m_pub_armed_heartbeat;

    // Subscribers
	rclcpp::Subscription<uav_msgs::msg::ArmAck>::SharedPtr m_sub_arm_ack;
	rclcpp::Subscription<uav_msgs::msg::DisarmAck>::SharedPtr m_sub_disarm_ack;
	rclcpp::Subscription<uav_msgs::msg::ArmedHeartbeat>::SharedPtr m_sub_armed_heartbeat;
};