#pragma once

#include <stdint.h>



// Enums

enum class MavMsgType : uint8_t {
    Ack, // When you send a command such as Arm, the Ack will tell you if it was successful
    Arm, // Arm UAV
    Disarm, // Disarm UAV
    ControlInput, // Sends throttle, yaw, pitch, roll
    ControlMode, // Set MavControlMode
    //Request, // Request a message such as IMU data
    IMU, // accelerometer, angular velocity
    Kinematics, // acc, vel, pos linearly and angular
};

enum class MavControlMode {
    //RC,
    GuiController,
    Autonomous,
};



// Message types

struct MavMsgAck {
    MavMsgType type; // Type of this message (Ack)
    MavMsgType ack_type; // Type of the message that sent ack, for example (MavMsgArm)
    bool success; // Did the message succeed or not
};

struct MavMsgArm {
    MavMsgType type;
};

struct MavMsgDisarm {
    MavMsgType type;
};

struct MavMsgControlInput {
    MavMsgType type;
    float throttle;
    float yaw;
    float pitch;
    float roll;
};

/*struct MavMsgRequest {
    MavMsgType type;
    MavMsgType request_type;
};*/

struct MavMsgRequest {
    MavMsgType type;
    MavMsgType request_type;
};

struct MavMsgIMU {
    MavMsgType type;

    // Linear acc
    float acc_forward;
    float acc_right;
    float acc_down;

    // Angular vel
    float ang_vel_forward;
    float ang_vel_right;
    float ang_vel_down;
};


struct MavMsgKinematics {
    MavMsgType type;
    struct FrameFRD { float f; float r; float d; }; // Note: This is a struct, not a parameter

    // Linear kinematics
    FrameFRD acc;
    FrameFRD vel;
    FrameFRD pos;

    // Angular kinematics
    FrameFRD ang_acc;
    FrameFRD ang_vel;
    FrameFRD ang_pos;
};