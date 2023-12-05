#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/manual_control/manual_control.h>

#include <optional>
#include <memory>
#include <thread>

using namespace mavsdk;

// Utility functions for mavsdk stuff.
// This makes main.cpp much cleaner and it is easier to work.

bool mav_check_args(int argc, char** argv);
bool mav_connect(Mavsdk& mavsdk, const char* address);
bool mav_find_system(Mavsdk& mavsdk, std::shared_ptr<System>& system);
bool mav_wait_until_ready(const Telemetry& telemetry);

bool mav_arm(const Action& action);
bool mav_disarm(const Action& action);

// This function is blocking
void mav_send_heartbeats(
    ManualControl& control,
    uint32_t beat_duration_ms, // the time of beat, for example 20 ms
    uint32_t total_time_ms); // the total time it will beat, for example 1000 ms (one second)

bool mav_start_altitude_control(const ManualControl& control);