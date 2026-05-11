#include "angle.h" 
#include <math.h>

#define RAD_TO_DEG 57.2957795f

float COMP_ALPHA = 0.985f;     /* gyro trust factor */
float g_angle = 0.0f;          /* current angle for external use */
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
    /* Step 2: accelerometer angle (use Y/Z plane) */
    float acc_angle = atan2f(imu.ay, imu.az) * RAD_TO_DEG;

    /* Step 3: complementary filter using gyro Y */
    imu.angle = COMP_ALPHA * (imu.angle + (imu.gy) * DT) +
                (1.0f - COMP_ALPHA) * acc_angle;

    /* Step 4: display flag every 10 ticks = 10 Hz */
    tick_count++;
    if (tick_count >= 10) {
        tick_count = 0;
        display_flag = 1;
    }

    /* Step 5: expose current angle for PSO */
    g_angle = imu.angle;
}
