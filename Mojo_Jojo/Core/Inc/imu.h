#ifndef IMU_H
#define IMU_H

#include "stm32f3xx_hal.h"

/* ---------- Compile-time axis configuration ---------- */
/* Flip ANGLE_SIGN to -1 if motor correction is inverted */
#define ANGLE_SIGN 1

/*
 * Sensor-fusion sign controls.
 * Use these to align accelerometer tilt and gyro rate directions.
 */
#define ACC_TILT_SIGN   -1.0f
#define GYRO_TILT_SIGN -1.0f

/* Raw diagnostic sign controls for Y axis. */
#define ACC_Y_SIGN    1.0f
#define GYRO_Y_SIGN   -1.0f

/*
 * Default angle computation uses atan2f(ax, az).
 * To swap axes, redefine ACCEL_ANGLE in imu.c.
 */

/* ---------- Sensor addresses and registers ---------- */
/* LSM303DLHC accelerometer (I2C) */
#define LSM303_ADDR_W        0x32
#define LSM303_ADDR_R        0x33
#define LSM303_CTRL_REG1_A   0x20
#define LSM303_CTRL_REG4_A   0x23
#define LSM303_OUT_X_L_A     0x28

/* L3GD20 gyroscope (SPI) */
#define GYRO_WHO_AM_I      0x0F
#define GYRO_WHO_AM_I_VAL  0xD4
#define GYRO_CTRL_REG1     0x20
#define GYRO_CTRL_REG4     0x23
#define GYRO_OUT_X_L       0x28

/* ---------- Data structure ---------- */
typedef struct {
	/* Raw readings */
	int16_t ax_raw, ay_raw, az_raw;
	int16_t gx_raw, gy_raw, gz_raw;

	/* Scaled values */
	float ax, ay, az;  /* g-units */
	float gx, gy, gz;  /* dps */

	/* Offsets (from calibration) */
	float ax_off, ay_off, az_off;
	float gx_off, gy_off, gz_off;

	/* Filtered angle */
	float angle;
} imu_data_t;

/* ---------- Public API ---------- */
void imu_init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi);
void imu_calibrate(uint16_t samples);
void imu_read_accel(void);
void imu_read_gyro(void);

/* Global IMU data, accessible from main and ISR */
extern volatile imu_data_t imu;

#endif /* IMU_H */
