#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <usart.h>

#include "i2cmaster.h"

#define PI	3.14159265359	/* Define Pi value */
#define Declination	0.0527/* Define declination of location from where measurement going to be done. e.g. here we have added declination from location Pune city, India. we can get it from http://www.magnetic-declination.com */

int offsetX = -22;
int offsetY = 105;

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
	float angle = atan(ratio) * 180.0 / ((float) PI);
	angle = angle < 0 ? angle * -1 : angle;
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
	float angle = atan(ratio) * 180.0 / ((float) PI);
	angle = angle < 0 ? angle * -1 : angle;
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

int main() {

	uart_init();
	io_redirect();

	// Initialize I2C communication
	i2c_init();
	Magneto_init();	

	int count = 0;
	while(1) {
		int heading = Magneto_GetHeading();
		int headingOffset = Magneto_GetHeadingOffset();

		int x = Magneto_GetX() + offsetX;
		int y = Magneto_GetY() + offsetY;
		int z = Magneto_GetZ();
		//printf("%d, %d, %d\n", x, y, z);
		printf("%d, %d, %d\n", x, y, z);

		count++;

		_delay_ms(50);
	}

	return 0;
}
