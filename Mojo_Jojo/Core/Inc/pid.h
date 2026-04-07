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
} pid_t;

void pid_init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float setpoint,
              float out_min,
              float out_max);
float pid_compute(pid_t *pid, float measurement, float dt);
void pid_reset(pid_t *pid);

#endif /* PID_H */