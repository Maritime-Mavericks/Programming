#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "usart.h"
#include "i2cmaster.h"
#include "minmea.h"

#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#define START 999
#define NEUTRAL 2999
#define END 4999

#define PI	3.14159265359	/* Define Pi value */
#define Declination	0.0527/* Define declination of location from where measurement going to be done */

void USART_init(void);
volatile char received_data;
char received_string[512]; // Maximum string length, adjust as needed
volatile uint8_t string_index = 0;

float latitude;
float longitude;
float speed;

// Right-most point of Alsik building (towards the castle)
float destLatitude = 54.914144;
float destLongitude = 9.783352;

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
	i2c_init(); // Initialize I2C communication
	
	Magneto_init();	

	DDRB |= (1<<PB1); // Set PB1 as output
	TCCR1A |= (1<<COM1A1) | (1<<WGM11); // Fast PWM, non-inverting mode
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Fast PWM, prescaler = 8
	ICR1 = 39999;   //20ms PWM period

	int startValueX;
	int startValueY;
	int neutralValueX;
	int neutralValueY;
	int endValueX;
	int endValueY;
	
	printf("\n");
	printf("Starting Calibration...\n");

	OCR1A = START; // Set position to 0 degrees
	_delay_ms(1000);
	for(int i = 0; i < 1; i++){
		startValueX = Magneto_GetX();
		startValueY = Magneto_GetY();
		printf("StartX: %d	StartY: %d\n", startValueX, startValueY);
		_delay_ms(500);
		OCR1A = NEUTRAL; // Set position to 90 degrees
		_delay_ms(1000);

		neutralValueX = Magneto_GetX();
		neutralValueY = Magneto_GetY();
		printf("NeutralX: %d	NeutralY: %d\n", neutralValueX, neutralValueY);
		_delay_ms(500);
		OCR1A = END; // Set position to 180 degrees (not neccessary)
		_delay_ms(1000);

		endValueX = Magneto_GetX();
		endValueY = Magneto_GetY();
		printf("EndX: %d	EndY: %d\n\n", endValueX, endValueY);
		_delay_ms(500);
		OCR1A = START;
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

	// INITIALIZE GPS
	// -------------------------------------------------------------------

	USART_init(); //Call the USART initialization code
    UCSR0B |= (1<<RXCIE0); //enable interrupts for RXIE
    sei(); //enable interrupts

	while(1){ // empty infinite loop

    }

	return 0;
}

ISR(USART_RX_vect){
	// Parse data
    received_data = UDR0;
    if (received_data == '\n') {
        // Parse formed line
        if(strstr(received_string, "GPR")){
            printf("%s\n", received_string);
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, received_string)) {
				latitude = minmea_tocoord(&frame.latitude);
				longitude = minmea_tocoord(&frame.longitude);
				speed = minmea_tofloat(&frame.speed);
                printf("Lat-Lon-Speed: %f -- %f -- %f\n\n", latitude, longitude, speed);
            }
            else {
                printf("Error! Not parsed correctly!");
            }
        }

        // Store a new line
        received_string[string_index] = '\0'; // Null-terminate the string
        string_index = 0; // Reset the string index
    } else {
        received_string[string_index++] = received_data;
    }

	// Calculate angles
	float heading = Magneto_GetHeadingOffset();
	float destHeading;
	float deltaLat = destLatitude - latitude; // y (kind of)
	float deltaLong = destLongitude - longitude; // x (kind of)
	float ratio = ((float)deltaLat ) / ((float)deltaLong);
	float angle = abs(atan(ratio) * 180.0 / ((float) PI));
	if(deltaLong <= 0 && deltaLat >= 0){
		destHeading = 90.0 - angle;
	} else if(deltaLong <= 0 && deltaLat <= 0){
		destHeading = 90.0 + angle;
	} else if(deltaLong >= 0 && deltaLat <= 0){
		destHeading = 270.0 - angle;
	} else if(deltaLong >= 0 && deltaLat >= 0){
		destHeading = 270.0 + angle;
	}

	// Find direction to rotate
	float destHeadMinusHead = destHeading - heading;
	if(destHeadMinusHead >= 0){ // case 1: heading < destHeading
		if(destHeadMinusHead <= 180.0){
			printf("Turn left\n");
		} else {
			printf("Turn right\n");
		}
	} else { // case 2: heading > destHeading
		if(destHeadMinusHead >= -180.0){
			printf("Turn left\n");
		} else {
			printf("Turn right\n");
		}
	}

}

void USART_init(void){
    UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALER);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
}