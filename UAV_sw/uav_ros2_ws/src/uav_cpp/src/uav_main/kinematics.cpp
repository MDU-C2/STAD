#include "kinematics.h"

#include <iostream>

Kinematics::Kinematics()
{
    // Initialize to 0
    m_linear = {};
    m_last_linear = {};
    m_angular = {};
    m_last_angular = {};
    m_last_timestamp_us_lin = 0;
    m_last_timestamp_us_ang = 0;
    //m_damp_factor = 0.95f;
    //m_trust_factor = 0.2f;
}

void Kinematics::add_acc_linear(Telemetry::AccelerationFrd acc, uint64_t timestamp_us)
{
    //if (std::abs(acc.forward_m_s2) < m_trust_factor) acc.forward_m_s2 = 0.0f;
    //if (std::abs(acc.right_m_s2) < m_trust_factor) acc.right_m_s2 = 0.0f;
    //if (std::abs(acc.down_m_s2) < m_trust_factor) acc.down_m_s2 = 0.0f;

    acc.down_m_s2 += 9.82f;
    m_linear.acc = acc;

    // Without this the first iteration gets messed up
    if (m_last_timestamp_us_lin == 0) {
        m_last_timestamp_us_lin = timestamp_us;
        return;
    }

    float delta_time_s = float(timestamp_us - m_last_timestamp_us_lin) / 100000.0f;
    integrate_linear(delta_time_s);

    m_last_timestamp_us_lin = timestamp_us;
}

void Kinematics::add_vel_angular(Telemetry::AngularVelocityFrd vel, uint64_t timestamp_us)
{
    m_angular.vel = vel;

    // Without this the first iteration gets messed up
    if (m_last_timestamp_us_ang == 0) {
        m_last_timestamp_us_ang = timestamp_us;
        return;
    }

    float delta_time_s = float(timestamp_us - m_last_timestamp_us_ang) / 100000.0f;
    integrate_linear(delta_time_s);

    m_last_timestamp_us_ang = timestamp_us;

    // Some filtering
    //m_linear.vel *= m_damp_factor;
    //m_linear.pos *= m_damp_factor;
    //m_angular.acc *= m_damp_factor;
    //m_angular.pos *= m_damp_factor;
}

KinVec3 Kinematics::get_acc_linear()
{
    return m_linear.acc;
}

KinVec3 Kinematics::get_vel_linear()
{
    return m_linear.vel;
}

KinVec3 Kinematics::get_pos_linear()
{
    return m_linear.pos;
}

KinVec3 Kinematics::get_acc_angular()
{
    return m_angular.acc;
}

KinVec3 Kinematics::get_vel_angular()
{
    return m_angular.vel;
}

KinVec3 Kinematics::get_pos_angular()
{
    return m_angular.pos;
}

KinMat3x3 Kinematics::get_mat_linear()
{
    return m_linear;
}

KinMat3x3 Kinematics::get_mat_angular()
{
    return m_angular;
}

void Kinematics::integrate_linear(float delta_time_s)
{
    // p = p0 + v * t
    m_linear.pos += m_linear.vel * delta_time_s;

    // v = v0 + a * t
    m_linear.vel += m_linear.acc * delta_time_s;

    m_last_linear = m_linear;
}

void Kinematics::integrate_angular(float delta_time_s)
{
    // a = (v - v0) / t
    m_angular.acc = (m_angular.vel - m_last_angular.vel) / delta_time_s;

    // p = p0 + v * t
    m_angular.pos += m_angular.vel * delta_time_s;

    m_last_angular = m_angular;
}

// Helper math stuff

KinVec3& KinVec3::operator=(const Telemetry::AccelerationFrd& other){
    forward = other.forward_m_s2;
    right = other.right_m_s2;
    down = other.down_m_s2;
    return *this;
}

KinVec3& KinVec3::operator=(const Telemetry::AngularVelocityFrd& other){
    forward = other.forward_rad_s;
    right = other.right_rad_s;
    down = other.down_rad_s;
    return *this;
}

KinVec3 KinVec3::operator+(const KinVec3& other){
    return {
        forward + other.forward,
        right + other.right,
        down + other.down
    };
}

KinVec3 KinVec3::operator-(const KinVec3& other){
    return {
        forward - other.forward,
        right - other.right,
        down - other.down
    };
}

KinVec3 KinVec3::operator*(const KinVec3& other){
    return {
        forward * other.forward,
        right * other.right,
        down * other.down
    };
}

KinVec3 KinVec3::operator/(const KinVec3& other){
    return {
        forward / other.forward,
        right / other.right,
        down / other.down
    };
}

KinVec3& KinVec3::operator+=(const KinVec3& other){
    forward += other.forward;
    right += other.right;
    down += other.down;
    return *this;
}

KinVec3& KinVec3::operator-=(const KinVec3& other){
    forward -= other.forward;
    right -= other.right;
    down -= other.down;
    return *this;
}

KinVec3& KinVec3::operator*=(const KinVec3& other){
    forward *= other.forward;
    right *= other.right;
    down *= other.down;
    return *this;
}

KinVec3& KinVec3::operator/=(const KinVec3& other){
    forward /= other.forward;
    right /= other.right;
    down /= other.down;
    return *this;
}