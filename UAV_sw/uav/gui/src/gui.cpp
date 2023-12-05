#include "gui.h"

#include <iostream>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

static GLFWwindow* g_window = NULL;

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

bool gui_init()
{
    glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return false;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	g_window = glfwCreateWindow(600, 400, "UAV GUI", NULL, NULL);
	if (!g_window)
	{
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(g_window);
	glfwSwapInterval(1); // VSync

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	if (!ImGui_ImplGlfw_InitForOpenGL(g_window, true)
		|| !ImGui_ImplOpenGL3_Init("#version 130"))
		return false;

    return true;
}

bool gui_is_window_open()
{
    return !glfwWindowShouldClose(g_window);
}

void gui_new_frame()
{
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGui::DockSpaceOverViewport();
}

void gui_render_frame()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(g_window);
}

void gui_destroy()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwDestroyWindow(g_window);
	glfwTerminate();
}