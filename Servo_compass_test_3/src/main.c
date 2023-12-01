#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>
#include <math.h>

#include "usart.h"
#include "i2cmaster.h"

#define START 999
#define NEUTRAL 2999
#define END 4999

#define PI	3.14159265359	/* Define Pi value */
#define Declination	0.0527/* Define declination of location from where measurement going to be done */

int offsetX = 0;
int offsetY = 0;

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

	printf("Starting 180 degree rotation with offset...\n");
	for(int i = 0; i < 1; i++){
		for(int angle = 0; angle <= 180; angle = angle + 5){
			int x = Magneto_GetX() + offsetX;
			int y = Magneto_GetY() + offsetY;
			int z = Magneto_GetZ();
			printf("%d, %d, %d\n", x, y, z);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
		for(int angle = 180; angle >= 0; angle = angle - 5){
			int x = Magneto_GetX() + offsetX;
			int y = Magneto_GetY() + offsetY;
			int z = Magneto_GetZ();
			printf("%d, %d, %d\n", x, y, z);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
	}
	printf("\n");

	_delay_ms(1000);

	printf("Starting 180 degree rotation with no offset...\n");
	for(int i = 0; i < 1; i++){
		for(int angle = 0; angle <= 180; angle = angle + 5){
			int x = Magneto_GetX();
			int y = Magneto_GetY();
			int z = Magneto_GetZ();
			printf("%d, %d, %d\n", x, y, z);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
		for(int angle = 180; angle >= 0; angle = angle - 5){
			int x = Magneto_GetX();
			int y = Magneto_GetY();
			int z = Magneto_GetZ();
			printf("%d, %d, %d\n", x, y, z);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
	}
	printf("\n");

	_delay_ms(1000);

	printf("Rotating 180 degrees...\n");
	OCR3A = START;
	_delay_ms(1000);
	int heading = Magneto_GetHeading();
	int headingOff = Magneto_GetHeadingOffset();
	printf("Angle: %d	AngleOff: %d\n", heading, headingOff);
	_delay_ms(1000);
	OCR3A = END;
	_delay_ms(1000);
	heading = Magneto_GetHeading();
	headingOff = Magneto_GetHeadingOffset();
	printf("Angle: %d	AngleOff: %d\n", heading, headingOff);
	_delay_ms(1000);
	OCR3A = START;
	_delay_ms(1000);
	printf("\n");
	
	printf("Trying to point towards north......\n");
	for(int angle = 0; angle <= 180; angle = angle + 5){
		OCR3A = START + ((((float) angle) / 180) * (END - START));
		_delay_ms(200);

		int x = Magneto_GetX() + offsetX;
		int y = Magneto_GetY() + offsetY;
		int z = Magneto_GetZ();
		int heading = Magneto_GetHeading();
		int headingOff = Magneto_GetHeadingOffset();
		printf("Angle: %d	AngleOff: %d	%d, %d, %d\n", heading, headingOff, x, y, z);

		if(abs(headingOff - 360) < 6 || abs(headingOff - 0) < 6){
			printf("FOUND NORTH! \n");
			break;
		}
	}

	_delay_ms(20000);

	OCR3A = START;
	_delay_ms(1000);

	return 0;
}