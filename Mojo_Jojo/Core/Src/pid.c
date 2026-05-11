#include "pid.h"

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float setpoint,
              float out_min,
              float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_min = out_min;
    pid->integral_max = out_max;
    pid->output = 0.0f;
}

float pid_compute(pid_controller_t *pid, float measurement, float dt)
{
    float error = pid->setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup clamp */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    }
    if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    float i_term = pid->ki * pid->integral;

    /* Derivative */
    float d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* Sum and clamp output */
    float out = p_term + i_term + d_term;
    if (out > pid->output_max) out = pid->output_max;
    if (out < pid->output_min) out = pid->output_min;

    pid->output = out;
    return out;
}

float pid_compute_speed(pid_controller_t *pid, float target_rpm, float actual_rpm, float dt)
{
    pid->setpoint = target_rpm;
    return pid_compute(pid, actual_rpm, dt);
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}