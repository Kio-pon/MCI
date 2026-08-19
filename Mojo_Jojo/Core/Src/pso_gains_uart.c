/*
 * pso_gains_uart.c
 *
 * Drop-in addition to your existing main.c / control loop.
 *
 * WHAT THIS DOES:
 *   The STM32 waits for a gain packet from Python.
 *   Python sends:  "GAINS 120.0 0.03 0.16 0.985\n"
 *   STM32 parses Kp, Ki, Kd, alpha, re-inits the balance PID.
 *   Tilt gate waits until robot is held vertical (±5° for 1 second).
 *   Then runs control loop for TRIAL_DURATION_MS milliseconds.
 *   Every 100ms it sends back the current angle over UART.
 *   After trial sends "DONE\n" or "FALL\n" and waits for next packet.
 *
 * PROTOCOL:
 *   PC -> STM32 :  "GAINS <Kp> <Ki> <Kd> <alpha>\n"
 *   STM32 -> PC :  "GAINS_OK\n"   (gains parsed and applied)
 *   STM32 -> PC :  "A <angle>\n"   (10 Hz during trial)
 *   STM32 -> PC :  "DONE\n"        (end of trial)
 *   STM32 -> PC :  "FALL\n"        (robot fell mid-trial)
 *
 * HOW TO INTEGRATE:
 *   1. Add this file to your project (CMakeLists.txt).
 *   2. In angle.h: change #define COMP_ALPHA to extern float COMP_ALPHA
 *   3. In angle.c: add float COMP_ALPHA = 0.985f; and g_angle assignment
 *   4. In main.c: replace pid_init() call with load_gains_and_run()
 */

#include "main.h"
#include "pid.h"
#include "angle.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Config                                                               */
/* ------------------------------------------------------------------ */
#define TRIAL_DURATION_MS   30000U   /* 30 second trial */
#define FALL_THRESHOLD_DEG  25.0f    /* treat this as a fall */
#define UART_BUF_LEN        64
#define DT                  0.01f    /* 10 ms control loop */

/* Tilt gate: wait until within this range for this duration */
#define TILT_GATE_THRESHOLD 5.0f     /* ±5 degrees */
#define TILT_GATE_DURATION  1000U    /* 1 second */

/* ------------------------------------------------------------------ */
/* Globals (extern these where needed)                                  */
/* ------------------------------------------------------------------ */
extern pid_controller_t balance_pid;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern volatile imu_data_t imu;
extern float g_setpoint;

/* ------------------------------------------------------------------ */
/* UART helpers                                                         */
/* ------------------------------------------------------------------ */
static char rx_buf[UART_BUF_LEN];
static char tx_buf[UART_BUF_LEN];

/* Blocking read of one '\n'-terminated line into rx_buf */
static void uart_readline(void)
{
    uint8_t c;
    uint8_t i = 0;
    memset(rx_buf, 0, UART_BUF_LEN);
    while (1) {
        if (HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY) == HAL_OK) {
            if (c == '\n' || i >= UART_BUF_LEN - 1) break;
            rx_buf[i++] = (char)c;
        }
    }
}

static void uart_send(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

/* ------------------------------------------------------------------ */
/* Parse "GAINS Kp Ki Kd alpha" from rx_buf                            */
/* Returns 1 on success, 0 on parse failure                            */
/* ------------------------------------------------------------------ */
static int parse_gains(float *kp, float *ki, float *kd, float *alpha)
{
    /* Expect format: "GAINS 120.0 0.03 0.16 0.985" */
    if (strncmp(rx_buf, "GAINS", 5) != 0) return 0;

    char *p = rx_buf + 5;   /* skip "GAINS" */
    char *end;

    *kp    = strtof(p, &end); if (end == p) return 0; p = end;
    *ki    = strtof(p, &end); if (end == p) return 0; p = end;
    *kd    = strtof(p, &end); if (end == p) return 0; p = end;
    *alpha = strtof(p, &end); if (end == p) return 0;

    return 1;
}

/* ------------------------------------------------------------------ */
/* Tilt gate: wait until robot is held vertical                        */
/* ------------------------------------------------------------------ */
static void wait_for_vertical(void)
{
    uint32_t stable_since = HAL_GetTick();
    
    uart_send("Waiting for robot vertical...\n");
    
    while (1) {
        angle_update();
        float angle = imu.angle;
        
        /* Check if within threshold */
        if (fabsf(angle) < TILT_GATE_THRESHOLD) {
            /* Still within threshold, check duration */
            if ((HAL_GetTick() - stable_since) >= TILT_GATE_DURATION) {
                uart_send("Ready. Starting trial.\n");
                break;
            }
        } else {
            /* Out of threshold, reset timer */
            stable_since = HAL_GetTick();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Main entry: call this instead of your fixed pid_init block           */
/* ------------------------------------------------------------------ */
void load_gains_and_run(void)
{
    float kp, ki, kd, alpha;

    while (1)   /* outer loop: one trial per gain packet */
    {
        /* 1. Wait for gain packet from Python */
        uart_readline();
        if (!parse_gains(&kp, &ki, &kd, &alpha)) {
            uart_send("ERR\n");
            continue;
        }

        /* 2. Apply gains with full output range */
        pid_init(&balance_pid,
                 kp, ki, kd,
                 0.0f,          /* setpoint = 0 degrees */
                 -4799.0f, 4799.0f);  /* full PWM range */

        uart_send("GAINS_OK\n");

        /* 3. Reset angle state */
        angle_init();
        g_setpoint = 0.0f;

        /* 4. Tilt gate: wait until vertical */
        wait_for_vertical();

        /* 5. Run trial */
        uint32_t start    = HAL_GetTick();
        uint32_t log_tick = start;
        uint8_t  fell     = 0;

        /* Enable TIM4 ISR */
        HAL_TIM_Base_Start_IT(&htim4);

        while ((HAL_GetTick() - start) < TRIAL_DURATION_MS)
        {
            /* Send angle at 10 Hz */
            if ((HAL_GetTick() - log_tick) >= 100) {
                log_tick = HAL_GetTick();
                snprintf(tx_buf, UART_BUF_LEN, "A %.3f\n", imu.angle);
                uart_send(tx_buf);
            }

            /* Fall detection */
            float a = imu.angle;
            if (a > FALL_THRESHOLD_DEG || a < -FALL_THRESHOLD_DEG) {
                fell = 1;
                break;
            }
        }

        /* 6. Stop motors and timer */
        HAL_TIM_Base_Stop_IT(&htim4);
        motor_set(0, 0);

        /* 7. Report result */
        if (fell)
            uart_send("FALL\n");
        else
            uart_send("DONE\n");
    }
}
