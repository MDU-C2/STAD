#include "win_kinematics.h"

#include <mutex>
#include <imgui/imgui.h>

std::mutex g_mutex;

struct FrameFrd{
    void set(std::array<float, 3>& vals){
        frd[0] = vals[0];
        frd[1] = vals[1];
        frd[2] = vals[2];
    }

    float frd[3];
};

FrameFrd g_lin_acc = {};
FrameFrd g_lin_vel = {};
FrameFrd g_lin_pos = {};
FrameFrd g_ang_acc = {};
FrameFrd g_ang_vel = {};
FrameFrd g_ang_pos = {};


static void kinematics_callback(const uav_msgs::msg::Kinematics::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    g_lin_acc.set(msg->acc);
    g_lin_vel.set(msg->vel);
    g_lin_pos.set(msg->pos);
    g_ang_acc.set(msg->ang_acc);
    g_ang_vel.set(msg->ang_vel);
    g_ang_pos.set(msg->ang_pos);
}

WinKinematics::WinKinematics(rclcpp::Node::SharedPtr node)
{
    m_pub_kinematics = node->create_publisher<uav_msgs::msg::Kinematics>("k", 10);
	m_sub_kinematics = node->create_subscription<uav_msgs::msg::Kinematics>("k", 10, kinematics_callback);
}

void WinKinematics::process()
{
    if (ImGui::Begin("Kinematics"))
    {
        ImGui::BeginGroup();
        ImGui::Text("Linear acceleration: ");
        ImGui::Text("Linear velocity: ");
        ImGui::Text("Linear position: ");
        ImGui::Text("Angular acceleration: ");
        ImGui::Text("Angular velocity: ");
        ImGui::Text("Angular position: ");
        ImGui::EndGroup();

        std::unique_lock<std::mutex> lock(g_mutex);

        for (int i = 0; i < 3; i++)
        {
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::Text(std::to_string(g_lin_acc.frd[i]).c_str());
            ImGui::Text(std::to_string(g_lin_vel.frd[i]).c_str());
            ImGui::Text(std::to_string(g_lin_pos.frd[i]).c_str());
            ImGui::Text(std::to_string(g_ang_acc.frd[i]).c_str());
            ImGui::Text(std::to_string(g_ang_vel.frd[i]).c_str());
            ImGui::Text(std::to_string(g_ang_pos.frd[i]).c_str());
            ImGui::EndGroup();
        }

        ImGui::End();
    }
}