#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>
#include <math.h>

#include "usart.h"
#include "i2cmaster.h"

#define MIN 999
#define NEUTRAL 2999
#define MAX 4999

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
	Heading = atan2((double)y,(double)x) + Declination;
	if (Heading>2*PI)	/* Due to declination check for >360 degree */
		Heading = Heading - 2*PI;
	if (Heading<0)		/* Check for sign */
		Heading = Heading + 2*PI;
	return (Heading* 180 / PI);/* Convert into angle and return */
}

int Magneto_GetHeadingOffset(){
	int x, y, z;
	double Heading;
	i2c_start_wait(0x3C);	/* Start and wait for acknowledgment */
	i2c_write(0x03);	/* Write memory location address */
	i2c_rep_start(0x3D);/* Generate repeat start condition with SLA+R */
	/* Read 16 bit x,y,z value (2's complement form) */
	x = (((int)i2c_readAck()<<8) | (int)i2c_readAck()) + offsetX;
	z = (((int)i2c_readAck()<<8) | (int)i2c_readAck()) + offsetY;
	y = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	i2c_read(0); // Must to a NAK on the last read
	i2c_stop();		/* Stop I2C */
	Heading = atan2((double)y,(double)x) + Declination;
	if (Heading>2*PI)	/* Due to declination check for >360 degree */
		Heading = Heading - 2*PI;
	if (Heading<0)		/* Check for sign */
		Heading = Heading + 2*PI;
	return (Heading* 180 / PI);/* Convert into angle and return */
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

	DDRB |= (1<<PB1); // Set PB1 as output

	TCCR1A |= (1<<COM1A1) | (1<<WGM11); // Fast PWM, non-inverting mode
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Fast PWM, prescaler = 8
	ICR1 = 39999;   //20ms PWM period

	OCR1A = MIN; // Set position to 0 degrees
	_delay_ms(1000);

	int minValueX;
	int minValueY;
	int neutralValueX;
	int neutralValueY;
	int maxValueX;
	int maxValueY;
	
	printf("\n");
	printf("Starting Calibration...\n\n");

	for(int i = 0; i < 1; i++){
		OCR1A = MIN; // Set position to 0 degrees
		minValueX = Magneto_GetX();
		minValueY = Magneto_GetY();
		_delay_ms(1000);

		OCR1A = NEUTRAL; // Set position to 90 degrees
		neutralValueX = Magneto_GetX();
		neutralValueY = Magneto_GetY();
		_delay_ms(1000);

		OCR1A = MAX; // Set position to 180 degrees
		maxValueX = Magneto_GetX();
		maxValueY = Magneto_GetY();
		_delay_ms(2000);
	}

	_delay_ms(1000);

	printf("Computing offsets...\n");
	int radius = abs(minValueX - maxValueX);
	if(minValueX < maxValueX){ // if minValueX is the left of the circle
		offsetX = -1 * (minValueX + (radius / 2));
	} else { // if minValueX is the right of the circle
		offsetX = -1 * (maxValueX + (radius / 2));
	}
	if(neutralValueY > minValueY){ // if neutralValueY is the top of the circle
		offsetY = -1 * (neutralValueY - (radius / 2));
	} else { // if neutralValueY is the bottom of the circle
		offsetY = -1 * (neutralValueY + (radius / 2));
	}
	printf("X-Offset: %d\n", offsetX);
	printf("Y-Offset: %d\n", offsetY);
	printf("\n");

	_delay_ms(1000);

	printf("Starting 360 degree rotation...\n");
	for(int i = 0; i < 2; i++){
		for(int angle = 0; angle <= 180; angle = angle + 5){
			int x = Magneto_GetX();
			int y = Magneto_GetY();
			int z = Magneto_GetZ();
			printf("%d, %d, %d\n", x, y, z);
			OCR1A = MIN + ((((float) angle) / 180) * (MAX - MIN));
			_delay_ms(200);
		}
		for(int angle = 180; angle >= 0; angle = angle - 5){
			int x = Magneto_GetX();
			int y = Magneto_GetY();
			int z = Magneto_GetZ();
			printf("%d, %d, %d\n", x, y, z);
			OCR1A = MIN + ((((float) angle) / 180) * (MAX - MIN));
			_delay_ms(200);
		}
	}

	return 0;
}