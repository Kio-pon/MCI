#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f3xx_hal.h"

#define ENCODER_PPR 330U  /* 11 pulses x 30:1 gearbox */

void encoder_init(TIM_HandleTypeDef *htim_counter);
float encoder_get_rpm_left(void);
float encoder_get_rpm_right(void);
void encoder_exti_handler(uint16_t GPIO_Pin);

#endif /* ENCODER_H */