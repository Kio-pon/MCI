#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f3xx_hal.h"

/*
 * motor_set(left, right)
 * left, right: -999 to +999
 * positive = forward, negative = backward
 * 0 = stop (coast)
 */
void motor_init(TIM_HandleTypeDef *htim);
void motor_set(int16_t left, int16_t right);
void motor_stop(void);

#endif /* MOTOR_H */
