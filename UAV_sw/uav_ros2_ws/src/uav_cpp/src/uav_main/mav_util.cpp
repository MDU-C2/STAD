#include "mav_util.h"

#include <iostream>

bool mav_check_args(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage : " << argv[1] << " <connection_url>\n"
                << "Connection URL format should be :\n"
                << " For TCP : tcp://[server_host][:server_port]\n"
                << " For UDP : udp://[bind_host][:bind_port]\n"
                << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
                << "For example, to connect to the simulator use URL: udp://:14540\n";
        return false;
    }
    return true;
}

bool mav_connect(Mavsdk& mavsdk, const char* address)
{
    std::cout << "Connecting...\n";
    ConnectionResult connection_result = mavsdk.add_any_connection(address);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return false;
    }
    return true;
}

bool mav_find_system(Mavsdk& mavsdk, std::shared_ptr<System>& system)
{
    std::cout << "Querying UAVs...\n";
    auto sys = mavsdk.first_autopilot(3.0);
    if (!sys.has_value())
    {
        std::cerr << "System timed out!\n";
        return false;
    }

    system = sys.value();
    return true;
}

bool mav_wait_until_ready(const Telemetry& telemetry)
{
    bool ready = false;

    for (int i = 0; !(ready = telemetry.health_all_ok()); i++) {
        std::cout << "Waiting for system to be ready...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    if (!ready) {
        std::cerr << "System timed out!\n";
        return false;
    }

    std::cout << "System is ready!\n";
    return true;
}

bool mav_arm(const Action& action)
{
    std::cout << "Arming...\n";
    auto action_result = action.arm();
    if (action_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << action_result << '\n';
        return false;
    }

    std::cout << "Armed!\n";
    return true;
}

bool mav_disarm(const Action& action)
{
    std::cout << "Disarming...\n";
    auto action_result = action.disarm();
    if (action_result != Action::Result::Success) {
        std::cerr << "Disarming failed: " << action_result << '\n';
        return false;
    }

    std::cout << "Disarmed!\n";
    return true;
}

void mav_send_heartbeats(
    ManualControl& control, 
    uint32_t beat_duration_ms, 
    uint32_t total_time_ms)
{
    uint32_t count = total_time_ms / beat_duration_ms;

    for (unsigned i = 0; i < count; ++i) {
        control.set_manual_control_input(0.f, 0.f, 0.5f, 0.f);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool mav_start_altitude_control(const ManualControl& control)
{
    std::cout << "starting altutude control...\n";
    auto manual_control_result = control.start_altitude_control();
    if (manual_control_result != ManualControl::Result::Success) {
        std::cerr << "Altutude control failed: " << manual_control_result << '\n';
        return false;
    }

    std::cout << "Started altutude control!\n";
    return true;
}