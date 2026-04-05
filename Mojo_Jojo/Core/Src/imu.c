#include "imu.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG  57.2957795f
#define ACC_SCALE   (3.9f / 1000.0f)
#define GYRO_SCALE  0.00875f

/* Macro for angle computation (change axes here if needed) */
// FOR x axis atanf 
#define ACCEL_ANGLE(d) (atan2f((d).ay, (d).az) * RAD_TO_DEG)

static I2C_HandleTypeDef *_hi2c;
static SPI_HandleTypeDef *_hspi;

volatile imu_data_t imu;

/* ---- internal helpers ---- */
static void gyro_cs_low(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

static void gyro_cs_high(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

static uint8_t gyro_read_reg(uint8_t reg)
{
	uint8_t tx = 0x80 | reg; /* bit 7 = read */
	uint8_t rx = 0;

	gyro_cs_low();
	HAL_SPI_Transmit(_hspi, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(_hspi, &rx, 1, HAL_MAX_DELAY);
	gyro_cs_high();

	return rx;
}

static void gyro_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = {reg, val};

	gyro_cs_low();
	HAL_SPI_Transmit(_hspi, tx, 2, HAL_MAX_DELAY);
	gyro_cs_high();
}

/* ---- public functions ---- */
void imu_init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi)
{
	_hi2c = hi2c;
	_hspi = hspi;
	memset((void *)&imu, 0, sizeof(imu));

	/* --- Gyroscope CS idle high --- */
	gyro_cs_high();

	/* --- LSM303AGR accelerometer init --- */
	uint8_t val;

	/* CTRL_REG1_A: ODR = 100 Hz, all axes enabled */
	val = 0x57; /* 0101 0111: ODR=100 Hz, LPen=0, Zen=Yen=Xen=1 */
	HAL_I2C_Mem_Write(_hi2c, LSM303_ADDR_W, LSM303_CTRL_REG1_A,
					  I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);

	/* CTRL_REG4_A: normal mode, +/-2g */
	val = 0x00;
	HAL_I2C_Mem_Write(_hi2c, LSM303_ADDR_W, LSM303_CTRL_REG4_A,
					  I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);

	/* --- I3G4250D gyroscope init --- */
	/* CTRL_REG1: PD=1, ODR=100 Hz, BW=25 Hz, all axes on */
	/* Binary: 00 00 1111 = 0x0F for 100 Hz ODR */
	/* Or use 0x8F for 800 Hz ODR */
	/* For 100 Hz control loop, 100 Hz ODR is fine */
	gyro_write_reg(GYRO_CTRL_REG1, 0x0F);

	(void)gyro_read_reg(GYRO_WHO_AM_I);
}

void imu_calibrate(uint16_t samples)
{
	float sx = 0, sy = 0, sz = 0;
	float sgx = 0, sgy = 0, sgz = 0;

	for (uint16_t i = 0; i < samples; i++) {
		imu_read_accel();
		imu_read_gyro();

		sx += imu.ax;
		sy += imu.ay;
		sz += imu.az;

		sgx += imu.gx;
		sgy += imu.gy;
		sgz += imu.gz;

		HAL_Delay(5);
	}

	/* Keep gravity in Z so tilt angle uses gravity as its reference vector. */
	imu.ax_off = sx / samples;
	imu.ay_off = sy / samples;
	imu.az_off = sz / samples - 1.0f;
	imu.gx_off = sgx / samples;
	imu.gy_off = sgy / samples;
	imu.gz_off = sgz / samples;
}

void imu_read_accel(void)
{
	uint8_t buf[6];

	/* Auto-increment read: 0x80 | 0x28 = 0xA8 */
	HAL_I2C_Mem_Read(_hi2c, LSM303_ADDR_R, 0xA8,
					 I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);

	imu.ax_raw = (int16_t)((buf[1] << 8) | buf[0]);
	imu.ay_raw = (int16_t)((buf[3] << 8) | buf[2]);
	imu.az_raw = (int16_t)((buf[5] << 8) | buf[4]);

    imu.ax = ((imu.ax_raw >> 6) * ACC_SCALE) - imu.ax_off;
    imu.ay = ((imu.ay_raw >> 6) * ACC_SCALE) - imu.ay_off;
    imu.az = ((imu.az_raw >> 6) * ACC_SCALE) - imu.az_off + 1.0f;
}

void imu_read_gyro(void)
{
	uint8_t tx;
	uint8_t buf[6];

	/* Burst read: set auto-increment bit (bit 6) + read bit (bit 7) */
	tx = 0xC0 | GYRO_OUT_X_L; /* 0xC0 | 0x28 = 0xE8 */

	gyro_cs_low();
	HAL_SPI_Transmit(_hspi, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(_hspi, buf, 6, HAL_MAX_DELAY);
	gyro_cs_high();

	imu.gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
	imu.gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
	imu.gz_raw = (int16_t)((buf[5] << 8) | buf[4]);

	imu.gx = (imu.gx_raw * GYRO_SCALE) - imu.gx_off;
	imu.gy = (imu.gy_raw * GYRO_SCALE) - imu.gy_off;
	imu.gz = (imu.gz_raw * GYRO_SCALE) - imu.gz_off;
}
