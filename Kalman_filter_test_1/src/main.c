#define F_CPU 16000000UL

#include <inttypes.h>		/* Include integer type header file */
#include <stdlib.h>			/* Include standard library file */
#include <stdio.h>			/* Include standard I/O library file */
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <usart.h>
#include <avr/interrupt.h>

#include "i2cmaster.h"
#include "MPU6050_res_define.h"	/* Include MPU6050 register define file */

#define PI 3.14159265
#define RAD_TO_DEG (180.0 / PI)

void Gyro_Init(void);
void MPU_Start_Loc(void);
void Read_RawValue(void);
void Timer0_init(void);

float Acc_x, Acc_y, Acc_z, Temperature, Gyro_x, Gyro_y, Gyro_z;
float Xa, Ya, Za, Xg = 0, Yg = 0, Zg = 0;
float offsetAX, offsetAY, offsetAZ, offsetGX, offsetGY, offsetGZ;
float xAngleGyr = 0, yAngleGyr = 0, xAngleAcc, yAngleAcc, zAngleAcc;

volatile uint16_t counter = 0;
volatile uint16_t counter2 = 0;

float KalmanAngleX = 0;
float KalmanAngleXUncertainty = 2*2;
float KalmanAngleY = 0;
float KalmanAngleYUncertainty = 2*2;
float KalmanAnglePrediction = 0;
float KalmanUncertaintyPrediction = 0;

float ComplimentaryAngleX = 0;
float ComplimentaryAngleY = 0;

void Calibrate_MPU6050();
void Calculate_Kalman(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, int milliseconds);

int main() {

	uart_init();
	io_redirect();
	i2c_init();
	Gyro_Init();
	Timer0_init();
	sei(); 

	// Calibrate accelerometer
	printf("Calibrating accelerometer and gyroscope...\n");
	Calibrate_MPU6050();
	printf("OffsetAX: %0.2f    OffsetAZ: %0.2f\n", offsetAX, offsetAZ);

	int inc = 0;
	int timeForOneIteration = 20;
	while(1){
		if(counter >= timeForOneIteration){ // in milliseconds
			Read_RawValue();

			Xa = (Acc_x/16384.0) + offsetAX;
			Ya = (Acc_y/16384.0) + offsetAY;
			Za = (Acc_z/16384.0) + offsetAZ;
			
			Xg = (Gyro_x/16.4) + offsetGX;
			Yg = (Gyro_y/16.4) + offsetGY;
			Zg = (Gyro_z/16.4) + offsetGZ;

			// Note that the x angle from accelerometer is how much the x axis rotates
			// So, it is the arctan of Y/Z! And for the y angle, it is arctan of X/Z.
			xAngleAcc = RAD_TO_DEG * atan2(-Ya, -Za); 
			yAngleAcc = -1 * RAD_TO_DEG * atan2(-Xa, -Za); 

			// Integrate gyro rate
			xAngleGyr += (timeForOneIteration * 0.001) * Xg;
			yAngleGyr += (timeForOneIteration * 0.001) * Yg;

			Calculate_Kalman(KalmanAngleX, KalmanAngleXUncertainty, Xg, xAngleAcc, timeForOneIteration);
			KalmanAngleX = KalmanAnglePrediction;
			KalmanAngleXUncertainty = KalmanUncertaintyPrediction;
			Calculate_Kalman(KalmanAngleY, KalmanAngleYUncertainty, Yg, yAngleAcc, timeForOneIteration);
			KalmanAngleY = KalmanAnglePrediction;
			KalmanAngleYUncertainty = KalmanUncertaintyPrediction;
			
			float alpha = 0.15;
			ComplimentaryAngleX = alpha * xAngleGyr + (1 - alpha) * xAngleAcc;
			ComplimentaryAngleY = alpha * yAngleAcc + (1 - alpha) * yAngleAcc;

			counter = 0;
		}

		if(counter2 >= 200){ // in milliseconds
			printf("%d, %0.2f, %0.2f, %0.2f, %0.2f\n", inc, yAngleAcc, yAngleGyr, ComplimentaryAngleY, KalmanAngleY);
			counter2 = 0;
			inc++;
		}
	}

	return 0;
}

ISR(TIMER0_COMPA_vect) {
    counter++;
	counter2++;
}

void Timer0_init(){
	TCCR0A |= (1 << WGM01);
    OCR0A = 0xF9; // equivalent of delay which is 1 millisecond (using formula)
    TCCR0B |= (1 << CS01) | (1 << CS00); // 64 prescaler
    TCNT0 = 0;
	TIMSK0 |= (1 << OCIE0A);
}

void Calculate_Kalman(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, int milliseconds){
	float standardDeviationGyro = 0.12;
	float standardDeviationAcc = 0.19;
	float deltaTime = milliseconds * 0.001;

	KalmanState += deltaTime * KalmanInput;
	KalmanUncertainty += deltaTime * deltaTime * (standardDeviationGyro * standardDeviationGyro);

	float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + (standardDeviationAcc * standardDeviationAcc));
	KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
	KalmanUncertainty *= (1 - KalmanGain);

	KalmanAnglePrediction = KalmanState; 
	KalmanUncertaintyPrediction = KalmanUncertainty;
}

void Calibrate_MPU6050(){
	float sumAccX = 0;
	float sumAccY = 0;
	float sumAccZ = 0;
	float sumGyrX = 0;
	float sumGyrY = 0;
	float sumGyrZ = 0;
	int count = 20;
	for(int i = 0; i < count; i++){
		Read_RawValue();

		Xa = (Acc_x/16384.0);
		Ya = (Acc_y/16384.0);
		Za = (Acc_z/16384.0);
	
		Xg = (Gyro_x/16.4);
		Yg = (Gyro_y/16.4);
		Zg = (Gyro_z/16.4);

		sumAccX += Xa;
		sumAccY += Ya;
		sumAccZ += Za;

		sumGyrX += Xg;
		sumGyrY += Yg;
		sumGyrZ += Zg;

		_delay_ms(50);
	}
	offsetAX = -1 * sumAccX / count;
	offsetAY = -1 * sumAccY / count;
	offsetAZ = 0;

	offsetGX = -1 * sumGyrX / count;
	offsetGY = -1 * sumGyrY / count;
	offsetGZ = -1 * sumGyrZ / count;
}

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





