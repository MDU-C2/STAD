#include "win_commands.h"

#include <imgui/imgui.h>
#include <iostream>
#include <string>

#include <uav/udp.h>
#include <uav/msgs.h>
#include <chrono>

using TimePoint = std::chrono::_V2::steady_clock::time_point;

struct Param {
    std::string str;
    TimePoint timepoint;
};

Param g_param_arm;
Param g_param_disarm;
Param g_param_control_mode;

char* g_str_arm;
char* g_str_disarm;
char* g_str_control_mode;

// Either manual or autonomous for now
bool g_manual_control_mode;

const char* param_handle_ack(WinCommandsAck& ack, const char* str_success, const char* str_fail)
{
    if (ack.is_valid)
    {
        ack.is_valid = false;
        if (ack.success)
            g_param_arm.str = str_success;
        else
            g_param_arm.str = str_fail;
    }
}

void win_commands(WinCommandsParams& params, std::mutex& mutex)
{
    mutex.lock();
    if (params.arm_heartbeat) {
        params.arm_heartbeat = false;
        g_str_arm = "Armed :)";
    }

    if (params.disarm_heartbeat) {
        params.disarm_heartbeat = false;
        g_str_disarm = "Disarmed :)";
    }

    g_str_arm = (char*)param_handle_ack(params.arm_ack, "Armed!", "Arming failed.");
    g_str_disarm = (char*)param_handle_ack(params.disarm_ack, "Disarmed!", "Disarming failed.");

    // Receive control mode ack
    if (params.control_mode_ack.is_valid)
    {
        params.control_mode_ack.is_valid = false;
        if (params.control_mode_ack.success)
        {
            if (g_manual_control_mode)
                g_param_control_mode.str = "Autonomous mode\n";
            else
                g_param_control_mode.str = "Manual mode\n";
            
            // switcheroo
            g_manual_control_mode = !g_manual_control_mode;
        }
    }

    mutex.unlock();

    if (ImGui::Begin("Commands"))
    {

        if (ImGui::Button("Arm UAV"))
        {
            g_param_arm.str = "Waiting...";
            MavMsgArm msg = { MavMsgType::Arm };
            udp_send_msg(&msg, sizeof(msg));
        }

        ImGui::SameLine();
        ImGui::Text(g_str_arm);

        if (ImGui::Button("Disarm UAV"))
        {
            g_param_disarm.str = "Waiting...";
            MavMsgDisarm msg = { MavMsgType::Disarm };
            udp_send_msg(&msg, sizeof(msg));
        }

        ImGui::SameLine();
        ImGui::Text(g_str_disarm);

        /*if (ImGui::Button("Disarm UAV"))
        {
            g_param_disarm.str = "Waiting...";
            MavMsgDisarm msg = { MavMsgType::Disarm };
            udp_send_msg(&msg, sizeof(msg));
        }

        ImGui::SameLine();
        ImGui::Text(g_param_arm.str.c_str());*/
    }
    ImGui::End();
}