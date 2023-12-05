#include "PID.h"

PID::PID(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_current_value = 0.0f;
    m_error_sum = 0.0f;
    m_last_error = 0.0f;
}

void PID::set_gains(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

}

void PID::set_gain_p(float kp)
{
    m_kp = kp;
}

void PID::set_gain_i(float ki)
{
    m_ki = ki;
}

void PID::set_gain_d(float kd)
{
    m_kd = kd;
}

float PID::update(float target, float dt)
{
    float error = target - m_current_value;
    m_error_sum += error * dt;

    float delta_error = (error - m_last_error) / dt;

    float output = m_kp * error + m_ki * m_error_sum + m_kd * delta_error;

    m_last_error = error;

    return output;
}