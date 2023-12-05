#include <iostream>
#include <string>
#include <mutex>
#include <signal.h>
#include <imgui/imgui.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "gui.h"
#include "win_controller.h"
#include "win_commands.h"
#include "win_kinematics.h"

bool g_exit = false;

// Ctrl + C in terminal
void sigint_handler(int sig)
{
	std::cout << "\nReceived signal: " << sig << "\n";
	g_exit = true;
}

void spin_task(rclcpp::Node::SharedPtr node)
{
	rclcpp::spin(node);
}

int main(int argc, char** argv)
{
	signal(SIGINT, sigint_handler);
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("uav_gui_node");
	//auto p7 = node->create_publisher<uav_msgs::msg::ControlMode>("cm", 10);

	WinController win_controller(node);
	WinCommands win_commands(node);
	WinKinematics win_kinematics(node);

	std::thread spin_thread(spin_task, node);

	// export DISPLAY=:0;

	if (!gui_init())
		return -1;

	while (gui_is_window_open() && !g_exit)
	{
		gui_new_frame(); // this must come before all rendering code

		// This is where all windows go.
		
		win_controller.process();

		win_commands.process();

		win_kinematics.process();

		
		ImGui::ShowDemoWindow();

		gui_render_frame(); // this must come after all rendering code
	}

	// Cleanup
	gui_destroy();
	rclcpp::shutdown();
	return 0;
}
