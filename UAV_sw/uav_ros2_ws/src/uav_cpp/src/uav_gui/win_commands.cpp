#include "win_commands.h"

#include <imgui/imgui.h>
#include <iostream>
#include <string>
#include <mutex>
#include <chrono>

static std::mutex g_mutex;

char* g_str_arm = "\0";
char* g_str_disarm = "\0";
char* g_str_control_mode = "\0";

int g_timestamp_arm;
int g_timestamp_disarm;
int g_timestamp_control_mode;

bool g_armed = false;
uint64_t g_heartbeat_counter = 0;

static void arm_ack_callback(const uav_msgs::msg::ArmAck::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    if (msg->success)
        g_str_arm = "Arming succeded :)";
    else
        g_str_arm = "Arming failed :(";
}

static void disarm_ack_callback(const uav_msgs::msg::DisarmAck::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    if (msg->success)
        g_str_disarm = "Disarming succeded :)";
    else
        g_str_disarm = "Disarming failed :(";
}

static void armed_heartbeat_callback(const uav_msgs::msg::ArmedHeartbeat::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    g_armed = msg->armed;
    g_heartbeat_counter++;
}

WinCommands::WinCommands(rclcpp::Node::SharedPtr node)
{
    m_pub_arm = node->create_publisher<uav_msgs::msg::Arm>("a", 10);
	m_pub_arm_ack = node->create_publisher<uav_msgs::msg::ArmAck>("aa", 10);
	m_pub_disarm = node->create_publisher<uav_msgs::msg::Disarm>("d", 10);
	m_pub_disarm_ack = node->create_publisher<uav_msgs::msg::DisarmAck>("da", 10);
	m_pub_armed_heartbeat = node->create_publisher<uav_msgs::msg::ArmedHeartbeat>("ah", 10);

	m_sub_arm_ack = node->create_subscription<uav_msgs::msg::ArmAck>("aa", 10, arm_ack_callback);
	m_sub_disarm_ack = node->create_subscription<uav_msgs::msg::DisarmAck>("da", 10, disarm_ack_callback);
	m_sub_armed_heartbeat = node->create_subscription<uav_msgs::msg::ArmedHeartbeat>("ah", 10, armed_heartbeat_callback);
}

void WinCommands::process()
{
    if (ImGui::Begin("Commands"))
    {

        g_mutex.lock();
        std::string str_arm_status = "status: ";
        if (g_armed)
            str_arm_status.append("Armed");
        else
            str_arm_status.append("Disarmed");
        g_mutex.unlock();
        ImGui::Text(str_arm_status.c_str());

        if (ImGui::Button("Arm UAV"))
        {
            g_mutex.lock();
            g_str_arm = "Waiting...";
            g_mutex.unlock();
            auto msg = uav_msgs::msg::Arm();
            auto duration = std::chrono::steady_clock::now().time_since_epoch();
            msg.time_point = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            m_pub_arm->publish(msg);
        }

        ImGui::SameLine();
        g_mutex.lock();
        ImGui::Text(g_str_arm);
        g_mutex.unlock();

        if (ImGui::Button("Disarm UAV"))
        {
            g_mutex.lock();
            g_str_disarm = "Waiting...";
            g_mutex.unlock();
            auto msg = uav_msgs::msg::Disarm();
            auto duration = std::chrono::steady_clock::now().time_since_epoch();
            msg.time_point = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            m_pub_disarm->publish(msg);
        }

        ImGui::SameLine();
        g_mutex.lock();
        ImGui::Text(g_str_disarm);
        std::string str_heartbeats = "Heartbeats received: " + std::to_string(g_heartbeat_counter);
        g_mutex.unlock();
        ImGui::Text(str_heartbeats.c_str());
    }
    ImGui::End();
}