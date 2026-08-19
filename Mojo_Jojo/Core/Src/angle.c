#include "angle.h"

#include <math.h>

#define RAD_TO_DEG 57.2957795f

volatile uint8_t display_flag = 0;
static uint16_t tick_count = 0;

void angle_init(void)
{
    imu.angle = 0.0f;
    display_flag = 0;
    tick_count = 0;
}

void angle_update(void)
{
    /* Step 1: read sensors */
    imu_read_accel();
    imu_read_gyro();

    /* Step 2: accelerometer angle */
    float acc_angle = atan2f(imu.ay, imu.az) * RAD_TO_DEG;

    /* Step 3: complementary filter */
    imu.angle = COMP_ALPHA * (imu.angle + (imu.gy) * DT) +
                (1.0f - COMP_ALPHA) * acc_angle;

    /* Step 4: display flag every 10 ticks = 10 Hz */
    tick_count++;
    if (tick_count >= 10) {
        tick_count = 0;
        display_flag = 1;
    }
}
