#ifndef ANGLE_H
#define ANGLE_H

#include "imu.h"

#define DT         0.01f  /* 10 ms, matches TIM4 at 100 Hz */
extern float COMP_ALPHA;  /* gyro trust factor (runtime variable for PSO) */
extern float g_angle;     /* current angle (exposed for PSO) */

void angle_init(void);
void angle_update(void);  /* called from TIM4 ISR */

extern volatile uint8_t display_flag;

#endif /* ANGLE_H */
