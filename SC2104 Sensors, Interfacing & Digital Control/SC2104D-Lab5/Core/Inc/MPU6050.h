#include <stdint.h> /* need for data type declarations */
#include "stm32f4xx_hal.h" /* Needed for I2C */
#include <stdbool.h>   // to support boolean data type

extern uint8_t IMU_ADDR;

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	UART_HandleTypeDef *uart;
	/* Acceleration data (X, Y, Z) in m/s^2 */
	float acc[3];
	float gyro[3];
	/* Temperature data in deg */
	float temp_C;
} IMU_Data;

#define REG_ADD_ACCEL_XOUT_H    0x3B
#define REG_ADD_ACCEL_XOUT_L    0x3C
#define REG_ADD_ACCEL_YOUT_H    0x3D
#define REG_ADD_ACCEL_YOUT_L    0x3E
#define REG_ADD_ACCEL_ZOUT_H    0x3F
#define REG_ADD_ACCEL_ZOUT_L    0x40
#define REG_ADD_GYRO_XOUT_H     0x43
#define REG_ADD_GYRO_XOUT_L     0x44
#define REG_ADD_GYRO_YOUT_H     0x45
#define REG_ADD_GYRO_YOUT_L     0x46
#define REG_ADD_GYRO_ZOUT_H     0x47
#define REG_ADD_GYRO_ZOUT_L     0x48
#define REG_ADD_TEMP_OUT_H      0x41
#define REG_ADD_TEMP_OUT_L      0x42

#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1   0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */ //Gyro Full Scale = +/- 500 degree per second
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */ // enable LPF
#define REG_ADD_ACCEL_SMPLRT_DIV_1  0x10
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */
#define REG_ADD_TEMP_CONFIG         0x53
#define REG_VAL_TEMP_CONFIG         0x00 //

static const uint8_t REG_WHO_AM_I = 0x75; // ID of  WHO_AM_I register in MPU6050



