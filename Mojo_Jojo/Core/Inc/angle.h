#ifndef ANGLE_H
#define ANGLE_H

#include "imu.h"

#define DT         0.01f  /* 10 ms, matches TIM4 at 100 Hz */
#define COMP_ALPHA 0.98f  /* gyro trust factor */

void angle_init(void);
void angle_update(void);  /* called from TIM4 ISR */

extern volatile uint8_t display_flag;

#endif /* ANGLE_H */
