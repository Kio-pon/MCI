#include "encoder.h"

static TIM_HandleTypeDef *_htim;

/* Volatile because modified in ISR, read from main */
static volatile uint32_t right_last_tick = 0;
static volatile uint32_t right_period = 0;
static volatile uint32_t left_last_tick = 0;
static volatile uint32_t left_period = 0;

void encoder_init(TIM_HandleTypeDef *htim_counter)
{
    _htim = htim_counter;
    HAL_TIM_Base_Start(_htim);
}

/* Called from HAL_GPIO_EXTI_Callback.
 * Computes period between consecutive rising edges.
 */
void encoder_exti_handler(uint16_t GPIO_Pin)
{
    uint32_t now = __HAL_TIM_GET_COUNTER(_htim);

    if (GPIO_Pin == GPIO_PIN_8) {
        /* Right encoder on PC8 */
        right_period = now - right_last_tick;
        right_last_tick = now;
    } else if (GPIO_Pin == GPIO_PIN_9) {
        /* Left encoder on PC9 */
        left_period = now - left_last_tick;
        left_last_tick = now;
    }
}

float encoder_get_rpm_right(void)
{
    if (_htim == NULL) return 0.0f;
    uint32_t now = __HAL_TIM_GET_COUNTER(_htim);
    uint32_t p = right_period;

    /* If stopped or hasn't ticked in 100ms (< 1.8 RPM), report 0 speed */
    if (p == 0 || (now - right_last_tick) > 100000) {
        return 0.0f;
    }

    /* TIM2 runs at 1 MHz, so p is in microseconds */
    float freq = 1000000.0f / (float)p;
    return (60.0f * freq) / (float)ENCODER_PPR;
}

float encoder_get_rpm_left(void)
{
    if (_htim == NULL) return 0.0f;
    uint32_t now = __HAL_TIM_GET_COUNTER(_htim);
    uint32_t p = left_period;

    if (p == 0 || (now - left_last_tick) > 100000) {
        return 0.0f;
    }

    float freq = 1000000.0f / (float)p;
    return (60.0f * freq) / (float)ENCODER_PPR;
}