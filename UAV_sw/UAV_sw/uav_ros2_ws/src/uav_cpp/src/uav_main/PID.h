#pragma once

#include <stdint.h>

class PID
{
public:
    PID() = default; // Initialzes everything to 0
    PID(float kp, float ki, float kd);

    void set_gains(float kp, float ki, float kd);
    void set_gain_p(float kp);
    void set_gain_i(float ki);
    void set_gain_d(float kd);

    float update(float target, float dt);

private:
    float m_kp;         // Proportional gain
    float m_ki;         // Integral gain
    float m_kd;         // Derivative gain

    float m_current_value;

    float m_error_sum;     // Sum of errors for integral term
    float m_last_error;     // Previous error for derivative term
};