#include <chrono>
#include <memory>
#include <iostream>
#include <optional>
#include <signal.h>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>

// messages
#include <uav_msgs/msg/arm.hpp>
#include <uav_msgs/msg/arm_ack.hpp>
#include <uav_msgs/msg/disarm.hpp>
#include <uav_msgs/msg/disarm_ack.hpp>
#include <uav_msgs/msg/armed_heartbeat.hpp>
#include <uav_msgs/msg/control_input.hpp>
#include <uav_msgs/msg/control_mode.hpp>
#include <uav_msgs/msg/kinematics.hpp>
#include <uav_msgs/msg/vision.hpp>

#include "mav_util.h"
#include "kinematics.h"
#include "PID.h"
#include "vision.h"

using namespace mavsdk;

TargetData g_target;
std::mutex g_vision_mutex;

void vision_callback(const uav_msgs::msg::Vision::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(g_vision_mutex);
    g_target = calculate_target_data(msg);
    print_target_data(g_target);
}

// This mutex is for synchronization between main and revc_msg_callback.
// Otherwise we can get problems if the access the same variables at the same time.
std::mutex g_mutex;

// ------- UAV Global Controls ------- 

Mavsdk* g_mavsdk = nullptr;
Action* g_action = nullptr;
Telemetry* g_telemetry = nullptr;
ManualControl* g_manual_control = nullptr;
Kinematics* g_kinematics = nullptr;

// UAV manual control
float g_throttle = 0.0f;
float g_roll = 0.0f;
float g_pitch = 0.0f;
float g_yaw = 0.0f;

// These are the publishers we need
rclcpp::Publisher<uav_msgs::msg::ArmAck>::SharedPtr         g_pub_arm_ack;
rclcpp::Publisher<uav_msgs::msg::DisarmAck>::SharedPtr      g_pub_disarm_ack; 

void arm_callback(const uav_msgs::msg::Arm::SharedPtr msg)
{
    auto ack = uav_msgs::msg::ArmAck();
    ack.time_point = msg->time_point;
    ack.success = mav_arm(*g_action);
    g_pub_arm_ack->publish(ack);
}

void disarm_callback(const uav_msgs::msg::Disarm::SharedPtr msg)
{
    auto ack = uav_msgs::msg::DisarmAck();
    ack.time_point = msg->time_point;
    ack.success = mav_disarm(*g_action);
    g_pub_disarm_ack->publish(ack);
}

void control_input_callback(const uav_msgs::msg::ControlInput::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    g_throttle = msg->throttle;
    g_roll = msg->roll;
    g_pitch = msg->pitch;
    g_yaw = msg->yaw;
}


// Ctrl + C in terminal
void sigint_handler(int sig)
{
	std::cout << "\nReceived signal: " << sig << "\n";
    rclcpp::shutdown();
	exit(-1);
}

void spin_task(rclcpp::Node::SharedPtr node)
{
	rclcpp::spin(node);
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigint_handler);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("uav_main_node");

    // By publishing to all nodes before subscibing we ensure
    // that they exist no matter the order we start the nodes
	auto p1 = node->create_publisher<uav_msgs::msg::Arm>("a", 10);
    auto p2 = node->create_publisher<uav_msgs::msg::Disarm>("d", 10);
    auto p3 = node->create_publisher<uav_msgs::msg::ControlInput>("ci", 10);
	auto p4 = node->create_publisher<uav_msgs::msg::ControlMode>("cm", 10);

    // These are the publishers we need
    g_pub_arm_ack = node->create_publisher<uav_msgs::msg::ArmAck>("aa", 10);
    g_pub_disarm_ack = node->create_publisher<uav_msgs::msg::DisarmAck>("da", 10);
    auto pub_armed_heartbeat = node->create_publisher<uav_msgs::msg::ArmedHeartbeat>("ah", 10);
	auto pub_kinematics = node->create_publisher<uav_msgs::msg::Kinematics>("k", 10);


    // These are the subscriptions we need
	auto sub_arm = node->create_subscription<uav_msgs::msg::Arm>("a", 10, arm_callback);
    auto sub_disarm = node->create_subscription<uav_msgs::msg::Disarm>("d", 10, disarm_callback);
    auto sub_control_input = node->create_subscription<uav_msgs::msg::ControlInput>("ci", 10, control_input_callback);
	//auto p7 = node->create_subscription<uav_msgs::msg::ControlMode>("cm", 10);

    auto sub_vision = node->create_subscription<uav_msgs::msg::Vision>("vision_topic", 10, vision_callback);

    std::thread spin_thread(spin_task, node);

    //std::this_thread::sleep_for(std::chrono::seconds(1000));

    Mavsdk mavsdk;
    g_mavsdk = &mavsdk;

    if (!mav_connect(mavsdk, "udp://192.168.0.4:14550"))
        return -1;
    
    std::shared_ptr<System> system;
    if (!mav_find_system(mavsdk, system))
        return -1;
 
    // Instantiate plugins.
    Action action = Action{system};
    g_action = &action;
    Telemetry telemetry = Telemetry{system};
    g_telemetry = &telemetry;
    ManualControl manual_control = ManualControl{system};
    g_manual_control = &manual_control;

    Kinematics kinematics;
    g_kinematics = &kinematics; 


    // IMU Subscription
    std::function<void(Telemetry::Imu)> imu_callback =
        [&](const Telemetry::Imu& msg) {
            kinematics.add_acc_linear(msg.acceleration_frd, msg.timestamp_us);
            kinematics.add_vel_angular(msg.angular_velocity_frd, msg.timestamp_us);
            
            auto msg2 = uav_msgs::msg::Kinematics();
            KinVec3 acc = kinematics.get_acc_linear();
            KinVec3 vel = kinematics.get_vel_linear();
            KinVec3 pos = kinematics.get_pos_linear();
            KinVec3 ang_acc = kinematics.get_acc_linear();
            KinVec3 ang_vel = kinematics.get_vel_linear();
            KinVec3 ang_pos = kinematics.get_pos_linear();
            memcpy(&msg2.acc, &acc, sizeof(float) * 3);
            memcpy(&msg2.vel, &vel, sizeof(float) * 3);
            memcpy(&msg2.pos, &pos, sizeof(float) * 3);
            memcpy(&msg2.ang_acc, &ang_acc, sizeof(float) * 3);
            memcpy(&msg2.ang_vel, &ang_vel, sizeof(float) * 3);
            memcpy(&msg2.ang_pos, &ang_pos, sizeof(float) * 3);
            pub_kinematics->publish(msg2);
        };
 
    Telemetry::ImuHandle imu_handle = telemetry.subscribe_imu(std::move(imu_callback));
 

    // Arming subscription
    std::function<void(bool)> armed_callback =
        [&](const bool armed) {
            auto msg = uav_msgs::msg::ArmedHeartbeat();
            msg.armed = armed;
            pub_armed_heartbeat->publish(msg);
        };

    Telemetry::ArmedHandle armed_handle = telemetry.subscribe_armed(std::move(armed_callback));

    std::cout << "Heartbeating before arming...\n";
    mav_send_heartbeats(manual_control, 20, 1200);

    if (!mav_arm(action))
        return -1;
    
    std::cout << "Preparing for manual control...\n";
    mav_send_heartbeats(manual_control, 20, 1200);
 
    if (!mav_start_altitude_control(manual_control))
        return -1;

    // Manual control from GUI application
    if (true)
    {
        while (true)
        {
            g_mutex.lock();
            manual_control.set_manual_control_input(g_pitch, g_roll, g_throttle, g_yaw);
            g_mutex.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // Autonomous flight
    else
    {
        // static PID pid_throttle(0.01f, 0.01f, 0.01f);
        // static PID pid_yaw(0.01f, 0.01f, 0.01f);
        // static PID pid_pitch(0.01f, 0.01f, 0.01f);
        // static PID pid_roll(0.01f, 0.01f, 0.01f);

        // Some algorithm is needed here.
        float throttle = 0.0f;
        while (true)
        {

            //pid_throttle.update()

            const float roll = 0.0f;
            const float pitch = 0.0f;
            const float yaw = 0.0f;
            g_mutex.lock();
            manual_control.set_manual_control_input(pitch, roll, throttle, yaw);
            g_mutex.unlock();
    
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
 
    std::cout << "Waiting for disarm...\n";
    while (telemetry.armed()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Disarmed!\n";
 
    telemetry.unsubscribe_imu(imu_handle);
    telemetry.unsubscribe_armed(armed_handle);
    rclcpp::shutdown();
    return 0;
}