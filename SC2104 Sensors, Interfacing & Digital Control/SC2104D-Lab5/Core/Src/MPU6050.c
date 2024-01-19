#include "MPU6050.h"
#include <string.h>   // for sprintf
#include <stdio.h>
#include <stdbool.h>   // to support boolean data type

HAL_StatusTypeDef ret; // to store return status
int16_t val; // data from IMU

HAL_StatusTypeDef IMU_WriteOneByte(IMU_Data *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMU_ReadOneByte(IMU_Data *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef Gyro_calibrate(IMU_Data *dev);

int16_t gyro_offset[3] = {0};  // gyro_offset value calibrated by Gyro_calibrate()

/*
 * INITIALISATION
 */
uint8_t* IMU_Initialise(IMU_Data *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uart)
{    // the uart is included in the parameter such that detailed message can be sent directly from within
     // the function here, instead of passing the information back to the main.c to send,  which can be messy.

	 char hex[2];
	 uint8_t uartbuf[20]="  IMU ID =      "; // buffer for data
     uint8_t regData;

	 dev->i2cHandle = i2cHandle;
	 dev->uart = uart;

	  //check ID
	  ret = IMU_ReadOneByte(dev, REG_WHO_AM_I, &regData);
	  if ( ret != HAL_OK )	  {
	       strcpy((char*)uartbuf, "Error 2\r\n");
	       return &uartbuf[0];
	       }
	  else{ // send ID read to Uart - for verififcation by user
		  sprintf(hex, "%x", regData); // change to hexidecimal
	      strcpy(uartbuf, hex); // copy back to buf
	      uartbuf[12] = uartbuf[0]; // change to upper case
	      uartbuf[13] = uartbuf[1];
	      uartbuf[14] = '\r';
	      uartbuf[15] = '\n';
	      uartbuf[16] = '\0';
	      uartbuf[0]  = '\r';
	      uartbuf[1]  = '\n';
	      uartbuf[2]  = 'I';
		  // for debuggiing - send to uart and return to main to display on OLED and UART
	      HAL_UART_Transmit(dev->uart, uartbuf, strlen((char*)uartbuf), HAL_MAX_DELAY);
	      }


	  // Initialize

      ret = IMU_WriteOneByte(dev, 0x1A,  0x05); //set lowpass filter
	  if ( ret != HAL_OK )	  {
	       strcpy((char*)uartbuf, "Error 1\r\n");
	       return &uartbuf[0];
	       }

      ret = IMU_WriteOneByte(dev, 0x1B,  0x08); // gyro senstitivity scales
	  if ( ret != HAL_OK )	  {
	       strcpy((char*)uartbuf, "Error 2\r\n");
	       return &uartbuf[0];
	       }

      ret = IMU_WriteOneByte(dev, 0x1C,  0x00); // Accelerator max scale +/- 2g
	  if ( ret != HAL_OK )	  {
	       strcpy((char*)uartbuf, "Error 3\r\n");
	       return &uartbuf[0];
	       }

      ret = IMU_WriteOneByte(dev, 0x6B, 0x00);  //Turn on power to IMU
	  if ( ret != HAL_OK )	  {
	       strcpy((char*)uartbuf, "Error 4\r\n");
	       return &uartbuf[0];
	       }

      HAL_Delay(10);

      // everthing OK
      return 0; // 0 means 0 error

}


HAL_StatusTypeDef IMU_WriteOneByte(IMU_Data *dev, uint8_t reg, uint8_t data)
{
	 uint8_t regData = data;
	 return HAL_I2C_Mem_Write(dev->i2cHandle, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &regData, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef IMU_ReadOneByte(IMU_Data *dev, uint8_t reg, uint8_t *data)
{
	ret=HAL_I2C_Mem_Read(dev->i2cHandle, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
	return ret;
}


HAL_StatusTypeDef IMU_AccelRead(IMU_Data *dev)
{
    uint8_t u8Buf[2] = {0}; // reset to zero
    int16_t accRaw[3] = {0};  // reset to zero

    ret=IMU_ReadOneByte(dev, 0x3c, &u8Buf[0]); // low byte of X
    ret=IMU_ReadOneByte(dev, 0x3b, &u8Buf[1]); // high byte
    accRaw[0] =	(u8Buf[1]<<8)|u8Buf[0];


    ret=IMU_ReadOneByte(dev, 0x3e, &u8Buf[0]); // Low byte of Y
    ret=IMU_ReadOneByte(dev, 0x3d, &u8Buf[1]);
    accRaw[1] =	(u8Buf[1]<<8)|u8Buf[0];


    ret=IMU_ReadOneByte(dev, 0x40, &u8Buf[0]); // low byte of Z
    ret=IMU_ReadOneByte(dev, 0x3f, &u8Buf[1]);
    accRaw[2] =	(u8Buf[1]<<8)|u8Buf[0];

	/* Convert to SIGNED integers (two's complement) */
	int32_t accRawSigned[3];

	if ( (accRaw[0] & 0x00008000) == 0x00008000 )
		accRawSigned[0] = accRaw[0] | 0xFFFF0000;
	else
		accRawSigned[0] = accRaw[0];

	if ( (accRaw[1] & 0x00008000) == 0x00008000 )
		accRawSigned[1] = accRaw[1] | 0xFFFF0000;
	else
		accRawSigned[1] = accRaw[1];

	if ( (accRaw[2] & 0x00008000) == 0x00008000 )
		accRawSigned[2] = accRaw[2] | 0xFFFF0000;
	else
		accRawSigned[2] = accRaw[2];


	// accelerometer full scale set to +/-2g, divide by sensitivity scale factor = 16384 LSB/g (see page 29, Registers 59-64)
	dev->acc[0] = 9.81f * 0.00006103515625f * accRawSigned[0];
	dev->acc[1] = 9.81f * 0.00006103515625f * accRawSigned[1];
	dev->acc[2] = 9.81f * 0.00006103515625f * accRawSigned[2];

	return ret;

}

HAL_StatusTypeDef Gyro_calibrate(IMU_Data *dev)  // calibrate the offset of the gyro
// store the offset in int16_t gyro_offset[3]
{
    uint8_t u8Buf[2] = {0}; // reset to zero upon entry
    int32_t gyroRaw[3] = {0}; // reset to zero upon entry
    int8_t i;
    int16_t temp;

    for (i=0; i< 32; i++){
    	IMU_ReadOneByte(dev, REG_ADD_GYRO_XOUT_L, &u8Buf[0]);
    	IMU_ReadOneByte(dev, REG_ADD_GYRO_XOUT_H, &u8Buf[1]);
    	temp = (u8Buf[1]<<8)|u8Buf[0]; // for debugging
    	gyroRaw[0] = temp + gyroRaw[0];
    	//gyroRaw[0] = (u8Buf[1]<<8)|u8Buf[0] + gyroRaw[0];

    	IMU_ReadOneByte(dev, REG_ADD_GYRO_YOUT_L, &u8Buf[0]);
    	IMU_ReadOneByte(dev, REG_ADD_GYRO_YOUT_H, &u8Buf[1]);
    	gyroRaw[1] = (u8Buf[1]<<8)|u8Buf[0] + gyroRaw[1];

    	IMU_ReadOneByte(dev, REG_ADD_GYRO_ZOUT_L, &u8Buf[0]);
    	ret=IMU_ReadOneByte(dev, REG_ADD_GYRO_ZOUT_H, &u8Buf[1]);
    	gyroRaw[2] = (u8Buf[1]<<8)|u8Buf[0] + gyroRaw[2];

    	HAL_Delay(100); // wait for 100msec
    }

    gyro_offset[0] = gyroRaw[0]>>5;  // average of 32 reads
    gyro_offset[1] = gyroRaw[1]>>5;
    gyro_offset[2] = gyroRaw[2]>>5;

	return ret;
}


HAL_StatusTypeDef IMU_GyroRead(IMU_Data *dev)
{   // return the change in value instead of current value
    uint8_t u8Buf[2] = {0}; // reset to zero
    int16_t gyroRaw[3] = {0};  // reset to zero
    int16_t gyroDiff[3];
    int16_t temp;
    static int16_t gyroOld[3]= {0, 0, 0};  // previous value

    ret=IMU_ReadOneByte(dev, 0x44, &u8Buf[0]); // gyro X lower byte
    ret=IMU_ReadOneByte(dev, 0x43, &u8Buf[1]); // upper byte
    temp = (u8Buf[1]<<8)|u8Buf[0]; // for debugging
    gyroRaw[0] = (u8Buf[1]<<8)|u8Buf[0] - gyro_offset[0];
    gyroDiff[0] = gyroRaw[0] - gyroOld[0];  // change in value
    gyroOld[0] = gyroRaw[0];

    ret=IMU_ReadOneByte(dev, 0x46, &u8Buf[0]);  // Gyro Y lower byte
    ret=IMU_ReadOneByte(dev, 0x45, &u8Buf[1]);  // upper byte
    gyroRaw[1] = (u8Buf[1]<<8)|u8Buf[0] -  gyro_offset[1];
    gyroDiff[1] = gyroRaw[1] - gyroOld[1];  // change in value
    gyroOld[1] = gyroRaw[1];

    ret=IMU_ReadOneByte(dev, 0x48, &u8Buf[0]);  // Gyro Z lower byte
    ret=IMU_ReadOneByte(dev, 0x47, &u8Buf[1]); // upper byte
    gyroRaw[2] = (u8Buf[1]<<8)|u8Buf[0] -  gyro_offset[2];
    gyroDiff[2] = gyroRaw[2] - gyroOld[2];  // change in value
    gyroOld[2] = gyroRaw[2];



	/* extend to 32 bit SIGNED integers (two's complement)*/
    int32_t gyroRawSigned[3];


	if ( (gyroRaw[0] & 0x00008000) == 0x00008000 )  //change to 32 bit
		gyroRawSigned[0] = gyroRaw[0] | 0xFFFF0000;
	else
		gyroRawSigned[0] = gyroRaw[0];

	if ( (gyroRaw[1] & 0x00008000) == 0x00008000 )
		gyroRawSigned[1] = gyroRaw[1] | 0xFFFF0000;
	else
		gyroRawSigned[1] = gyroRaw[1];

	if ( (gyroRaw[2] & 0x00008000) == 0x800008000 )
		gyroRawSigned[2] = gyroRaw[2] | 0xFFFF0000;
	else
		gyroRawSigned[2] = gyroRaw[2];


	// gyro full scale set to +/-500 dps, divide by sensitivity scale factor = 65.5 LSB/dps (see pg 31, Registers 67-72)
	// degree per second = value/65.5
	dev->gyro[0] = 0.0152671755725191f * gyroRawSigned[0];
	dev->gyro[1] = 0.0152671755725191f * gyroRawSigned[1];
	dev->gyro[2] = 0.0152671755725191f * gyroRawSigned[2];

	return ret;

}
