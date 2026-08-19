#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float output;
} pid_controller_t;

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float setpoint,
              float out_min,
              float out_max);
float pid_compute(pid_controller_t *pid, float measurement, float dt);
float pid_compute_speed(pid_controller_t *pid, float target_rpm, float actual_rpm, float dt);
void pid_reset(pid_controller_t *pid);

#endif /* PID_H */