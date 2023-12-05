#pragma once

#include <mutex>

struct WinCommandsAck {
    bool is_valid; // We only read success if this is valid.
    bool success;
};

struct WinCommandsParams {
    bool arm_heartbeat;     // True when heartbeat is received
    bool disarm_heartbeat;  // Heartbeats are sent all the time
    WinCommandsAck arm_ack; // True when ack is received
    WinCommandsAck disarm_ack; // Acks are sent as a response to a command
    WinCommandsAck control_mode_ack;
};

// This window handles all the commands related to the UAV. The
// udp message handler is in main.cpp so while this function is
// capable of sending messages, the acks must be fed in as a parameter
void win_commands(WinCommandsParams& params, std::mutex& mutex);
// This function also needs mutex because the params struct
// is shared with the msg receiver thread