#include "win_controller.h"

#include <imgui/imgui.h>
#include <iostream>

// Some math stuff
template <typename T>
T min(T a, T b) {
    return (a < b) ? a : b;
}

template <typename T>
T max(T a, T b) {
    return (a > b) ? a : b;
}

template <typename T>
T clamp(T val, T min, T max) {
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

template <typename T>
T abs(T val) {
    return (val < T(0)) ? -val : val;
}

template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}





struct Vec2 {
    float x;
    float y;
    Vec2 operator+(const Vec2& o) { return { x + o.x, y + o.y }; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2 operator*(const Vec2& o) { return { x * o.x, y * o.y }; }
};

typedef Vec2 Joystick;

struct JoystickPair {
    Joystick left;
    Joystick right;

    JoystickPair operator-() { return { { -left.x, -left.y }, { -right.x, -right.y } }; }
    JoystickPair operator+(const JoystickPair& o) { return { left + o.left, right + o.right }; }
    JoystickPair& operator+=(const JoystickPair& o) { left += o.left; right += o.right; return *this; }
    JoystickPair operator*(const JoystickPair& o) { return { left * o.left, right * o.right }; }
};

// Global stuff

JoystickPair g_joystick_pair = {};
JoystickPair g_axes_limits = {
    { 0.5f, 0.0f },
    { 0.2f, 0.2f }
};
JoystickPair g_axes_vel = {
    { 0.02f, 0.01f },
    { 0.03f, 0.03f}
};

float g_left_y_min = 0.0f;
float g_left_y_max = 1.0f;


static void draw_pad_target(Joystick joystick, const float threshRadius)
{
	const ImU32 white_color = IM_COL32(255, 255, 255, 255);
	
    // Get current rendering position on screen.
	const ImVec2 draw_pos = ImGui::GetCursorScreenPos();
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	
    // Draw "safe" region.
	draw_list->AddRectFilled(draw_pos, ImVec2(draw_pos.x + 200, draw_pos.y + 200), IM_COL32(0, 30, 60, 255));
	draw_list->AddCircleFilled(ImVec2(draw_pos.x + 100, draw_pos.y + 100), threshRadius, IM_COL32(0, 0, 0, 255), 32);
	
    // Draw frame and cross lines.
	draw_list->AddRect(draw_pos, ImVec2(draw_pos.x + 200, draw_pos.y + 200), white_color);
	draw_list->AddLine(ImVec2(draw_pos.x + 100, draw_pos.y), ImVec2(draw_pos.x + 100, draw_pos.y + 200), white_color);
	draw_list->AddLine(ImVec2(draw_pos.x, draw_pos.y + 100), ImVec2(draw_pos.x + 200, draw_pos.y + 100), white_color);
	
    // Draw threshold and unit radius circles.
	draw_list->AddCircle(ImVec2(draw_pos.x + 100, draw_pos.y + 100), threshRadius, IM_COL32(0, 130, 255, 255), 32);
	draw_list->AddCircle(ImVec2(draw_pos.x + 100, draw_pos.y + 100), 100, white_color, 32);
	
    // Current axis position.
	draw_list->AddCircleFilled(ImVec2(draw_pos.x + joystick.x * 100 + 100, draw_pos.y + joystick.y * 100 + 100), 10, white_color);
}

static void get_user_input()
{
    JoystickPair dirs;
    dirs.left.x = (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_D)) - (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_A));
    dirs.left.y = (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_S)) - (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_W));
    dirs.right.x = (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_RightArrow)) - (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_LeftArrow));
    dirs.right.y = (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_DownArrow)) - (int)ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_UpArrow));
    
    g_joystick_pair += dirs * g_axes_vel;

    g_joystick_pair.left.x =  clamp(g_joystick_pair.left.x,  -g_axes_limits.left.x,  g_axes_limits.left.x);
    g_joystick_pair.left.y =  clamp(g_joystick_pair.left.y,  -g_left_y_min,          g_left_y_max);
    g_joystick_pair.right.x = clamp(g_joystick_pair.right.x, -g_axes_limits.right.x, g_axes_limits.right.x);
    g_joystick_pair.right.y = clamp(g_joystick_pair.right.y, -g_axes_limits.right.y, g_axes_limits.right.y);
    
    // yes, should be 3 here, not 4
    if (dirs.left.x == 0.0f)
        g_joystick_pair.left.x -= min(abs(g_joystick_pair.left.x), g_axes_vel.left.x) * sign(g_joystick_pair.left.x);

    if (dirs.right.x == 0.0f)
        g_joystick_pair.right.x -= min(abs(g_joystick_pair.right.x), g_axes_vel.right.x) * sign(g_joystick_pair.right.x);

    if (dirs.right.y == 0.0f)
        g_joystick_pair.right.y -= min(abs(g_joystick_pair.right.y), g_axes_vel.right.y) * sign(g_joystick_pair.right.y);
}

WinController::WinController(rclcpp::Node::SharedPtr node)
{
    m_pub_control_input = node->create_publisher<uav_msgs::msg::ControlInput>("ci", 10);
}

void WinController::process()
{
    if (ImGui::Begin("Controller"))
    {
        get_user_input();

        const float threshRadius = 100.0f;

        // Titles.
        ImGui::Text("Threshold");
        ImGui::SameLine(100);
        ImGui::Text("Left pad & trigger");
        ImGui::SameLine(300 + 100);
        ImGui::Text("Right pad & trigger");

        // Left pad.
        ImGui::BeginChild("Pad Left Target", ImVec2(200, 200));
        draw_pad_target(g_joystick_pair.left, threshRadius);
        ImGui::EndChild();
        ImGui::SameLine();

        // Left stick vertical limits
        ImGui::BeginGroup();
        ImGui::VSliderFloat("##lsvlu", ImVec2(20, 100), &g_left_y_min, 0.0f, 1.0f, "%.2f");
        ImGui::VSliderFloat("##lsvll", ImVec2(20, 100), &g_left_y_max, 1.0f, 0.0f, "%.2f");
        ImGui::EndGroup();

        constexpr float max_vel = 0.03f;

        // Left stick vertical velocity
        ImGui::SameLine();
        ImGui::VSliderFloat("##lsvv", ImVec2(20, 200), &g_axes_vel.left.y, 0.0f, max_vel, "%.3f");
        ImGui::SameLine(300);

        // Right pad.
        ImGui::BeginChild("Pad Right Target", ImVec2(200, 200));
        draw_pad_target(g_joystick_pair.right, threshRadius);
        ImGui::EndChild();
        ImGui::SameLine();

        // Right stick vertical limits
        ImGui::VSliderFloat("##rsvl", ImVec2(20, 200), &g_axes_limits.right.y, 0.0f, 1.0f, "%.2f");

        // Right stick vertical velocity
        ImGui::SameLine();
        ImGui::VSliderFloat("##rsvv", ImVec2(20, 200), &g_axes_vel.right.y, 0.0f, max_vel, "%.3f");

        // Horizontal sliders

        // Left stick horizontal limits and velocities
        ImGui::BeginGroup();
        ImGui::SetNextItemWidth(200);
        ImGui::SliderFloat("##lshl", &g_axes_limits.left.x, 0.0f, 1.0f, "%.2f");
        ImGui::SetNextItemWidth(200);
        ImGui::SliderFloat("##lshv", &g_axes_vel.left.x, 0.0f, max_vel, "%.3f");
        ImGui::EndGroup();

        ImGui::SameLine(300);

        // Left stick horizontal limits and velocities
        ImGui::BeginGroup();
        ImGui::SetNextItemWidth(200);
        ImGui::SliderFloat("##rshl", &g_axes_limits.right.x, 0.0f, 1.0f, "%.2f");
        ImGui::SetNextItemWidth(200);
        ImGui::SliderFloat("##rshv", &g_axes_vel.right.x, 0.0f, max_vel, "%.3f");
        ImGui::EndGroup();
    }

    ImGui::End();

    const JoystickPair& joy = g_joystick_pair;

    // send input to uav
    auto msg = uav_msgs::msg::ControlInput();
    msg.throttle = (-joy.left.y + 1.0f) * 0.5f;
    msg.yaw = joy.left.x;
    msg.pitch = -joy.right.y;
    msg.roll = joy.right.x;
    m_pub_control_input->publish(msg);
}