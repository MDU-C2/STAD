#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <optional>
#include <signal.h>
#include <mutex>

#include <uav/udp.h>
#include <uav/msgs.h>

#include "mav_util.h"
#include "kinematics.h"
#include "PID.h"

using namespace mavsdk;

// This mutex is for synchronization between main and revc_msg_callback.
// Otherwise we can get problems if the access the same variables at the same time.
std::mutex g_mutex;

// ******* UAV Global Controls ******* \\

Mavsdk* g_mavsdk = nullptr;
Action* g_action = nullptr;
Telemetry* g_telemetry = nullptr;
ManualControl* g_manual_control = nullptr;

Kinematics* g_kinematics;

// UAV manual control
float g_throttle = 0.0f;
float g_roll = 0.0f;
float g_pitch = 0.0f;
float g_yaw = 0.0f;

void revc_msg_callback(void* buffer, size_t size)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    MavMsgType type = *((MavMsgType*)buffer);
    switch (type)
    {
        case MavMsgType::Arm:
        {
            MavMsgAck msg = {};
            msg.type = MavMsgType::Ack;     // This is an ACK,
            msg.ack_type = MavMsgType::Arm; // from arm command
            msg.success = mav_arm(*g_action);
            udp_send_msg(&msg, sizeof(msg));
            break;
        }
        case MavMsgType::Disarm:
        {
            MavMsgAck msg = {};
            msg.type = MavMsgType::Ack;         // This is an ACK,
            msg.ack_type = MavMsgType::Disarm;  // from disarm command
            msg.success = mav_disarm(*g_action);
            udp_send_msg(&msg, sizeof(msg));
            break;
        }
        case MavMsgType::ControlInput:
        {
            MavMsgControlInput* msg = ((MavMsgControlInput*)buffer);
            g_throttle = msg->throttle;
            g_roll = msg->roll;
            g_pitch = msg->pitch;
            g_yaw = msg->yaw;
            break;
        }
    }
}

// Ctrl + C in terminal
void sigint_handler(int sig)
{
	std::cout << "\nReceived signal: " << sig << "\n";
	udp_destroy();
	exit(-1);
}

int main(int argc, char** argv)
{

    signal(SIGINT, sigint_handler);

    if (argc == 3) {
        std::cout << "UDP input detected. Initializing...\n";
        //if (!udp_create_client(std::stoi(argv[2]), revc_msg_callback))
        //    return -1;
        //signal(SIGINT, sigint_handler);
    }
    //else
    //    std::cout << "No UDP input. Skipping...\n";

    if (!udp_create(UDP_Type::Client, revc_msg_callback))
        return -1;

    //if (!mav_check_args(argc, argv))
    //    return -1;

    Mavsdk mavsdk;
    g_mavsdk = &mavsdk;

    //if (!mav_connect(mavsdk, argv[1]))
    if (!mav_connect(mavsdk,"udp://:14540"))
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
        };
 
    Telemetry::ImuHandle imu_handle = telemetry.subscribe_imu(std::move(imu_callback));
 

    // Arming subscription
    std::function<void(bool)> armed_callback =
        [&](const bool armed) {
            if (armed) {
                MavMsgArm msg = { MavMsgType::Arm };
                udp_send_msg(&msg, sizeof(msg));
            } else {
                MavMsgDisarm msg = { MavMsgType::Disarm };
                udp_send_msg(&msg, sizeof(msg));
            }
        };

    Telemetry::ArmedHandle armed_handle = telemetry.subscribe_armed(std::move(armed_callback));

    std::cout << "Heartbeating before arming...\n";
    mav_send_heartbeats(manual_control, 20, 1200);

    if (!mav_arm(action))
        return -1;
    
    std::cout << "Preparing for manual control...\n";
    mav_send_heartbeats(manual_control, 20, 1200);
 
    //auto manual_control_result = manual_control.start_position_control();
    if (!mav_start_altitude_control(manual_control))
        return -1;

    // GUI control
    if (true)
    {
        while (true)
        {


            g_mutex.lock();
            manual_control.set_manual_control_input(g_pitch, g_roll, g_throttle, g_yaw);
            g_mutex.unlock();

            std::cout << "g_throttle: " << g_throttle << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // Autonomous
    else
    {
        PID pid_throttle(0.01f, 0.01f, 0.01f);
        PID pid_yaw(0.01f, 0.01f, 0.01f);
        PID pid_pitch(0.01f, 0.01f, 0.01f);
        PID pid_roll(0.01f, 0.01f, 0.01f);

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
 
    return 0;
}