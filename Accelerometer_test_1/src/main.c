#define F_CPU 16000000UL

#include <inttypes.h>		/* Include integer type header file */
#include <stdlib.h>			/* Include standard library file */
#include <stdio.h>			/* Include standard I/O library file */
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <usart.h>

#include "i2cmaster.h"
#include "MPU6050_res_define.h"	/* Include MPU6050 register define file */

#define PI 3.14159265
#define RAD_TO_DEG (180.0 / PI)

float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;

float offsetAX = -0.1;
float offsetAY = 0.01;
float offsetAZ = 0.1;

void Gyro_Init(){			/* Gyro initialization function */
	_delay_ms(150);			/* Power up time >100ms */
	i2c_start_wait(0xD0);	/* Start with device write address */
	i2c_write(SMPLRT_DIV);	/* Write to sample rate register */
	i2c_write(0x07);		/* 1KHz sample rate */
	i2c_stop();

	i2c_start_wait(0xD0);
	i2c_write(PWR_MGMT_1);	/* Write to power management register */
	i2c_write(0x01);		/* X axis gyroscope reference frequency */
	i2c_stop();

	i2c_start_wait(0xD0);
	i2c_write(CONFIG);		/* Write to Configuration register */
	i2c_write(0x00);		/* Fs = 8KHz */
	i2c_stop();

	i2c_start_wait(0xD0);
	i2c_write(GYRO_CONFIG);	/* Write to Gyro configuration register */
	i2c_write(0x18);		/* Full scale range +/- 2000 degree/C */
	i2c_stop();

	i2c_start_wait(0xD0);
	i2c_write(INT_ENABLE);	/* Write to interrupt enable register */
	i2c_write(0x01);
	i2c_stop();
}

void MPU_Start_Loc(){
	i2c_start_wait(0xD0);		/* I2C start with device write address */
	i2c_write(ACCEL_XOUT_H);	/* Write start location address from where to read */ 
	i2c_rep_start(0xD1);	/* I2C start with device read address */
}

void Read_RawValue(){
	MPU_Start_Loc();			/* Read Gyro values */
	Acc_x = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	Acc_y = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	Acc_z = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	Temperature = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	Gyro_x = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	Gyro_y = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	Gyro_z = (((int)i2c_readAck()<<8) | (int)i2c_readNak());
	i2c_stop();
}

int main() {

	uart_init();
	io_redirect();

	// Initialize I2C communication
	i2c_init();

	float Xa,Ya,Za,t;
	float Xg=0,Yg=0,Zg=0;

	Gyro_Init();

	while(1){
		Read_RawValue();

		/* Divide raw value by sensitivity scale factor */
		Xa = (Acc_x/16384.0) + offsetAX;
		Ya = (Acc_y/16384.0) + offsetAY;
		Za = (Acc_z/16384.0) + offsetAZ;
		
		Xg = Gyro_x/16.4;
		Yg = Gyro_y/16.4;
		Zg = Gyro_z/16.4;

		/* Convert temperature in /c using formula */
		t = (Temperature/340.00)+36.53;

		float x = RAD_TO_DEG * (atan2(-Xa, -Za) + PI); 
		float y = RAD_TO_DEG * (atan2(-Xa, -Za) + PI); 
		float z = RAD_TO_DEG * (atan2(-Ya, -Xa) + PI);
		
		//printf("Acceleration X = %f\nAcceleration Y = %f\nAcceleration Z = %f\n\n\n", Xa, Ya, Za);
		printf("Angle X = %f\nAngle Y = %f\nAngle Z = %f\n\n\n", x, y, z);

		_delay_ms(500);
	}

	return 0;
}








