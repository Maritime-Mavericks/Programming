#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>
#include <math.h>

#include "usart.h"
#include "i2cmaster.h"
#include "MPU6050_res_define.h"	/* Include MPU6050 register define file */

#define START 999
#define NEUTRAL 2999
#define END 4999

#define PI	3.14159265359	/* Define Pi value */
#define Declination	0.0527/* Define declination of location from where measurement going to be done */

#define RAD_TO_DEG (180.0 / PI)

float Acc_x, Acc_y, Acc_z, Temperature, Gyro_x, Gyro_y, Gyro_z;

float offsetAX = -0.08;
float offsetAY = 0.01;
float offsetAZ = 0;

int offsetX = 0;
int offsetY = 0;

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

void Magneto_init(){		/* Magneto initialize function */
	i2c_start(0x3C);	/* Start and write SLA+W */
	i2c_write(0x00);	/* Write memory location address */
	i2c_write(0x70);	/* Configure register A as 8-average, 15 Hz default, normal measurement */
	i2c_write(0xA0);	/* Configure register B for gain */
	i2c_write(0x00);	/* Configure continuous measurement mode in mode register */
	i2c_stop();		/* Stop I2C */
}

int Magneto_GetHeading(){
	int x = Magneto_GetX();
	int y = Magneto_GetY();
	float ratio = ((float)y ) / ((float)x);
	float angle = abs(atan(ratio) * 180.0 / ((float) PI));
	if(x >= 0 && y >= 0){ // quadrant 1
		return angle;
	} else if(x <= 0 && y >= 0){ // quadrant 2
		return (180.0 - angle);
	} else if(x <= 0 && y <= 0){ // quadrant 3
		return (180.0 + angle);
	} else if(x >= 0 && y <= 0){ // quadrant 4
		return (360.0 - angle);
	}
}

int Magneto_GetHeadingOffset(){
	int x = Magneto_GetX() + offsetX;
	int y = Magneto_GetY() + offsetY;
	float ratio = ((float)y ) / ((float)x);
	float angle = abs(atan(ratio) * 180.0 / ((float) PI));
	if(x >= 0 && y >= 0){ // quadrant 1
		return angle;
	} else if(x <= 0 && y >= 0){ // quadrant 2
		return (180.0 - angle);
	} else if(x <= 0 && y <= 0){ // quadrant 3
		return (180 + angle);
	} else if(x >= 0 && y <= 0){ // quadrant 4
		return (360 - angle);
	}
}

int Magneto_GetX(){
	int x, y, z;
	double Heading;
	i2c_start_wait(0x3C);	/* Start and wait for acknowledgment */
	i2c_write(0x03);	/* Write memory location address */
	i2c_rep_start(0x3D);/* Generate repeat start condition with SLA+R */
	/* Read 16 bit x,y,z value (2's complement form) */
	x = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	z = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	y = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	i2c_read(0); // Must to a NAK on the last read
	i2c_stop();		/* Stop I2C */
	return x;
}

int Magneto_GetY(){
	int x, y, z;
	double Heading;
	i2c_start_wait(0x3C);	/* Start and wait for acknowledgment */
	i2c_write(0x03);	/* Write memory location address */
	i2c_rep_start(0x3D);/* Generate repeat start condition with SLA+R */
	/* Read 16 bit x,y,z value (2's complement form) */
	x = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	z = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	y = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	i2c_read(0); // Must to a NAK on the last read
	i2c_stop();		/* Stop I2C */
	return y;
}

int Magneto_GetZ(){
	int x, y, z;
	double Heading;
	i2c_start_wait(0x3C);	/* Start and wait for acknowledgment */
	i2c_write(0x03);	/* Write memory location address */
	i2c_rep_start(0x3D);/* Generate repeat start condition with SLA+R */
	/* Read 16 bit x,y,z value (2's complement form) */
	x = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	z = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	y = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	i2c_read(0); // Must to a NAK on the last read
	i2c_stop();		/* Stop I2C */
	return z;
}

int main(void){

	uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

	// Initialize I2C communication
	i2c_init();
	Magneto_init();	
	Gyro_Init();

	DDRE |= (1<<PE3); // Set PB5 as output
	TCCR3A |= (1<<COM3A1) | (1<<WGM31); // Fast PWM, non-inverting mode
	TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31); // Fast PWM, prescaler = 8
	ICR3 = 39999;   //20ms PWM period

	int startValueX;
	int startValueY;
	int neutralValueX;
	int neutralValueY;
	int endValueX;
	int endValueY;
	
	printf("\n");
	printf("Starting Calibration...\n");

	OCR3A = START; // Set position to 0 degrees
	_delay_ms(1000);
	for(int i = 0; i < 1; i++){
		startValueX = Magneto_GetX();
		startValueY = Magneto_GetY();
		printf("StartX: %d	StartY: %d\n", startValueX, startValueY);
		_delay_ms(500);
		OCR3A = NEUTRAL; // Set position to 90 degrees
		_delay_ms(1000);

		neutralValueX = Magneto_GetX();
		neutralValueY = Magneto_GetY();
		printf("NeutralX: %d	NeutralY: %d\n", neutralValueX, neutralValueY);
		_delay_ms(500);
		OCR3A = END; // Set position to 180 degrees (not neccessary)
		_delay_ms(1000);

		endValueX = Magneto_GetX();
		endValueY = Magneto_GetY();
		printf("EndX: %d	EndY: %d\n\n", endValueX, endValueY);
		_delay_ms(500);
		OCR3A = START;
		_delay_ms(1000);
	}

	_delay_ms(1000);

	printf("Computing offsets...\n");
	int deltaX = endValueX - startValueX;
	int deltaY = endValueY - startValueY;
	offsetX = -1 * (startValueX + (deltaX / 2));
	offsetY = -1 * (startValueY + (deltaY / 2));
	
	printf("X-Offset: %d\n", offsetX);
	printf("Y-Offset: %d\n", offsetY);
	printf("\n");

	_delay_ms(1000);

	printf("Giving time for tilt...\n\n");
	_delay_ms(5000);
	
	printf("Starting 180 degree rotation...\n");
	for(int i = 0; i < 2; i++){
		for(int angle = 0; angle <= 90; angle = angle + 5){
			int x = Magneto_GetX() + offsetX;
			int y = Magneto_GetY() + offsetY;
			int z = Magneto_GetZ();
			int head = Magneto_GetHeadingOffset();

			Read_RawValue();
			/* Divide raw value by sensitivity scale factor */
			float Xa = (Acc_x/16384.0) + offsetAX;
			float Ya = (Acc_y/16384.0) + offsetAY;
			float Za = (Acc_z/16384.0) + offsetAZ;
			/* Convert temperature in /c using formula */
			float t = (Temperature/340.00)+36.53;
			float xAccAngle = RAD_TO_DEG * (atan2(-Xa, -Za)); // pitch
			float yAccAngle = RAD_TO_DEG * (atan2(-Ya, -Za)); // roll
			float zAccAngle = RAD_TO_DEG * (atan2(-Ya, -Xa)); // heading (acc)

			printf("%d, %d, %d, %0.2f, %0.2f\n", x, y, z, xAccAngle, yAccAngle);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
		for(int angle = 90; angle >= 0; angle = angle - 5){
			int x = Magneto_GetX() + offsetX;
			int y = Magneto_GetY() + offsetY;
			int z = Magneto_GetZ();
			int head = Magneto_GetHeadingOffset();

			Read_RawValue();
			/* Divide raw value by sensitivity scale factor */
			float Xa = (Acc_x/16384.0) + offsetAX;
			float Ya = (Acc_y/16384.0) + offsetAY;
			float Za = (Acc_z/16384.0) + offsetAZ;
			/* Convert temperature in /c using formula */
			float t = (Temperature/340.00)+36.53;
			float xAccAngle = RAD_TO_DEG * (atan2(-Xa, -Za)); // pitch
			float yAccAngle = RAD_TO_DEG * (atan2(-Ya, -Za)); // roll
			float zAccAngle = RAD_TO_DEG * (atan2(-Ya, -Xa)); // heading (acc)

			printf("%d, %d, %d, %0.2f, %0.2f\n", x, y, z, xAccAngle, yAccAngle);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
	}
	printf("\n");

	_delay_ms(1000);

	OCR3A = START;
	_delay_ms(1000);

	return 0;
}