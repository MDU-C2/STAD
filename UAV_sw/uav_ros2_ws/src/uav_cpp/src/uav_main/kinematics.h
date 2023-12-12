#pragma once

#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;

// Keeps track of some kinematic quantity in the frd frame.
// Like a 3 dimensional vector but better suiting names
struct KinVec3;

// Contains Kinematic vectors (acc, vel, pos)
struct KinMat3x3;

// This class integrates acc, vel and pos over time
// both linearly and angular so the UAV knows
// its transform in the world.

// The idea was to read IMU data which provides acceleration
// and then calculate velocity and position from that. In practice
// this works very bad and this should not be used.
class Kinematics;

struct KinVec3
{
    float forward;
    float right;
    float down;

    // Initializers
    KinVec3() = default;
    KinVec3(float v) : forward(v), right(v), down(v) {}
    KinVec3(float f, float r, float d) : forward(f), right(r), down(d) {}

    // Custom operators
    KinVec3& operator=(const Telemetry::AccelerationFrd& other);
    KinVec3& operator=(const Telemetry::AngularVelocityFrd& other);
    KinVec3 operator+(const KinVec3& other);
    KinVec3 operator-(const KinVec3& other);
    KinVec3 operator*(const KinVec3& other);
    KinVec3 operator/(const KinVec3& other);
    KinVec3& operator+=(const KinVec3& other);
    KinVec3& operator-=(const KinVec3& other);
    KinVec3& operator*=(const KinVec3& other);
    KinVec3& operator/=(const KinVec3& other);
};

struct KinMat3x3
{
    KinVec3 acc;
    KinVec3 vel;
    KinVec3 pos;
};

class Kinematics
{
public:
    Kinematics();

    void add_acc_linear(Telemetry::AccelerationFrd acc, uint64_t timestamp_us);
    void add_vel_angular(Telemetry::AngularVelocityFrd vel, uint64_t timestamp_us);

    KinVec3 get_acc_linear();
    KinVec3 get_vel_linear();
    KinVec3 get_pos_linear();

    KinVec3 get_acc_angular();
    KinVec3 get_vel_angular();
    KinVec3 get_pos_angular();

    KinMat3x3 get_mat_linear();
    KinMat3x3 get_mat_angular();

private:
    void integrate_linear(float delta_time_s);
    void integrate_angular(float delta_time_s);

    KinMat3x3 m_linear;
    KinMat3x3 m_last_linear;

    KinMat3x3 m_angular;
    KinMat3x3 m_last_angular;

    uint64_t m_last_timestamp_us_lin;
    uint64_t m_last_timestamp_us_ang;

    float m_damp_factor;
    float m_trust_factor;
};