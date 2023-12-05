#include <iostream>
#include <string>
#include <mutex>
#include <signal.h>

#include <imgui/imgui.h>
#include <uav/udp.h>
#include <uav/msgs.h>

#include "gui.h"
#include "win_controller.h"
#include "win_commands.h"

// This mutex is for synchronization between main and revc_msg_callback.
// Otherwise we can get problems if the access the same variables at the same time.
std::mutex g_mutex;

bool g_exit = false;

WinCommandsParams g_acks = {};

void recv_msg_callback(void* buffer, size_t size)
{
	std::unique_lock<std::mutex> lock(g_mutex);
    MavMsgType type = *((MavMsgType*)buffer);
    switch (type) 
	{
		case MavMsgType::Ack:
		{
			// Handle all acks
			MavMsgAck* msg = ((MavMsgAck*)buffer);
			switch (msg->ack_type) {
				case MavMsgType::Arm:
					g_acks.arm_ack = { true, msg->success };
					break;
				case MavMsgType::Disarm:
					g_acks.disarm_ack = { true, msg->success };
					break;
				case MavMsgType::ControlMode:
					g_acks.control_mode_ack = { true, msg->success };
					break;
			}
			break;
		}
		case MavMsgType::Arm:
		{
			break;
		}
		case MavMsgType::Kinematics:
		{
			MavMsgKinematics* msg = ((MavMsgKinematics*)buffer);
			msg->acc;
			break;
		}
    }
}

// Ctrl + C in terminal
void sigint_handler(int sig)
{
	std::cout << "\nReceived signal: " << sig << "\n";
	g_exit = true;
}

int main(int argc, char** argv)
{
	signal(SIGINT, sigint_handler);
	// export DISPLAY=:0;

	/*if (argc != 2) {
		std::cerr << "Expected a udp port as argument. exiting...\n";
		return -1;
	}*/

	// std::stoi(argv[1]);
	//if (!udp_create(UDP_Type::Server, recv_msg_callback))
	//	return -1;

	if (!gui_init())
		return -1;

	while (gui_is_window_open() && !g_exit)
	{
		gui_new_frame(); // this must come before all rendering code

		// This is where all windows go.
		
		win_controller();

		win_commands(g_acks, g_mutex);
		
		ImGui::ShowDemoWindow();

		gui_render_frame(); // this must come after all rendering code
	}

	// Cleanup
	gui_destroy();
	//udp_destroy();
	return 0;
}
