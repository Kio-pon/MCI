#include "motor.h"

static TIM_HandleTypeDef *_htim;

/* --- Right motor: PWM on TIM3_CH1 (PB4), dir on PC0/PC1 --- */
/* --- Left motor:  PWM on TIM3_CH2 (PB5), dir on PC6/PC7 --- */

/* TB6612FNG truth table:
 * IN1=H, IN2=L, PWM -> Forward
 * IN1=L, IN2=H, PWM -> Reverse
 * IN1=L, IN2=L      -> Coast (stop)
 */

static void set_right(int16_t speed)
{
    if (speed > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        speed = -speed;
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    if (speed > 999) speed = 999;
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, (uint16_t)speed);
}

static void set_left(int16_t speed)
{
    if (speed > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        speed = -speed;
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    if (speed > 999) speed = 999;
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, (uint16_t)speed);
}

void motor_init(TIM_HandleTypeDef *htim)
{
    _htim = htim;
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2);
    motor_stop();
}

void motor_set(int16_t left, int16_t right)
{
    set_left(left);
    set_right(right);
}

void motor_stop(void)
{
    set_left(0);
    set_right(0);
}
