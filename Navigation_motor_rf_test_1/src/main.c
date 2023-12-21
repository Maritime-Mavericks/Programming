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

#define MAX_FORWARD 1800// pwm max cw thrust (1900us)
#define MAX_REVERSE 1400 //pwm max ccw (1100us)
#define STOP 1500 // pwm stop(1500us)
#define CUTOFF 1600 // forward cutoff for proportional controller

volatile char received_data;
char received_string[512]; // Maximum string length, adjust as needed
volatile uint8_t string_index = 0;
volatile int boatCoordinateReceived = 0;
volatile float latitude;
volatile float longitude;
volatile float speed;

volatile int digitNumber = 0;
volatile int latOrLong = 0; // 0 is undefined, 17 is latitude, 31 is longitude
volatile int latDigits[8];
volatile int longDigits[7];
float destLatitude = 0;
float destLongitude = 0;

int offsetX = 0;
int offsetY = 0;

volatile int startRescueTransmission = 0;

void USART_initRF(void);
void USART_initGPS(void);
void Timer1_Init(void);
void Set_Thrust(int pulse_width, int pulse_width2); // right then left motor
void Magneto_init(void);
int Magneto_GetHeading(void);
int Magneto_GetHeadingOffset(void);
int Magneto_GetX(void);
int Magneto_GetY(void);
int Magneto_GetZ(void);
void Calibrate_Magnetometer(void);
int getPWMReduction(float deltaAngle);

int main(void){

	uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication
	i2c_init(); // Initialize I2C communication
	Magneto_init();	

	DDRB |= (1 << PB5); // right
    DDRB |= (1 << PB6); // left

	DDRE |= (1<<PE3); // Set PB5 as output
	TCCR3A |= (1<<COM3A1) | (1<<WGM31); // Fast PWM, non-inverting mode
	TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31); // Fast PWM, prescaler = 8
	ICR3 = 39999;   //20ms PWM period

	// Timer1 initialize
	printf("\nRunning motors soon...\n");
	Timer1_Init(); 
    //Initialize
    Set_Thrust(STOP,STOP); 
    _delay_ms(1000); 
    // Run the motor forward for 5 seconds
    _delay_ms(3000);
    _delay_ms(300); 
    Set_Thrust(MAX_FORWARD,MAX_FORWARD);
    _delay_ms(3000);
    Set_Thrust(STOP,STOP);

	printf("\nStarting Calibration...\n");
	Calibrate_Magnetometer();
	printf("X-Offset: %d\n", offsetX);
	printf("Y-Offset: %d\n", offsetY);
	printf("\n");

	_delay_ms(1000);

	// INITIALIZE GPS and RF
	// -------------------------------------------------------------------

	USART_initGPS(); //Call the USART initialization code
    UCSR3B |= (1<<RXCIE3); //enable interrupts for RXIE

	USART_initRF();
	UCSR2B |= (1<<RXCIE2); //enable interrupts for RXIE

    sei(); //enable interrupts

	while(1){ // empty infinite loop
		if(startRescueTransmission == 1){
			if(boatCoordinateReceived == 1){
				// Hard-code for testing
				latitude = 54.912726;
				longitude = 9.779898;

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
				int turnRightOrLeft = 0; // 0 is right, left is 1
				float destHeadMinusHead = destHeading - heading;
				float headMinusDestHead = heading - destHeading;
				float deltaAngle = 0;
				if(destHeadMinusHead >= 0){ // CASE 1 
					if(destHeadMinusHead >= 180){ 
						// turn right
						turnRightOrLeft = 0;
						deltaAngle = 360 - destHeadMinusHead;
					} else {
						// turn left
						turnRightOrLeft = 1;
						deltaAngle = destHeadMinusHead;
					}
				} else { // CASE 2
					if(headMinusDestHead >= 180){ 
						// turn left
						turnRightOrLeft = 1;
						deltaAngle = 360 - headMinusDestHead;
					} else {
						// turn right
						turnRightOrLeft = 0;
						deltaAngle = headMinusDestHead;
					}
				}

				printf("DestLat: %f    DestLong: %f\n", destLatitude, destLongitude);
				printf("heading: %0.0f, destHead: %0.0f\n", heading, destHeading);

				int pwmReduction = getPWMReduction(deltaAngle);
				if(turnRightOrLeft == 0){ // If turn right, make right motor lesser. Set left to max.
					printf("Turn right\n");
					Set_Thrust(MAX_FORWARD - pwmReduction, MAX_FORWARD);
				} else { // If turn left, make left motor lesser. Set right to max.
					printf("Turn left\n");
					Set_Thrust(MAX_FORWARD, MAX_FORWARD - pwmReduction);
				}

				printf("\n");
				boatCoordinateReceived = 0;
			}
		}
    }

	return 0;
}

ISR(USART2_RX_vect){
    received_data = UDR2;
    int digit = received_data - 48;

	if(digit == 35){ // start
		startRescueTransmission = 1;
	}

	if(startRescueTransmission == 1){
		if(digit == 17){ // latitude
			latOrLong = 17;
			digitNumber = 0;
		} else if(digit == 31){ // longitude 
			latOrLong = 31;
			digitNumber = 0;
		} else if(digit == 22){ // finished
			latOrLong = 0;
		} else { // if you received any other digit which would be the coordinate digits
			if(latOrLong == 17){ // latitude
				latDigits[digitNumber] = digit;
				digitNumber++;
			} else if(latOrLong == 31){ // longitude
				longDigits[digitNumber] = digit;
				digitNumber++;
			}
		}

		if(digit == 22){
			float receivedLatitude = 0;
			float receivedLongitude = 0;

			// Compute latitude
			receivedLatitude += (
				latDigits[0] * 10 +
				latDigits[1] * 1 +
				latDigits[2] * 0.1 +
				latDigits[3] * 0.01 +
				latDigits[4] * 0.001 +
				latDigits[5] * 0.0001 +
				latDigits[6] * 0.00001 + 
				latDigits[7] * 0.000001
			);

			// Compute longitude
			receivedLongitude += (
				longDigits[0] * 1 +
				longDigits[1] * 0.1 +
				longDigits[2] * 0.01 +
				longDigits[3] * 0.001 +
				longDigits[4] * 0.0001 +
				longDigits[5] * 0.00001 +
				longDigits[6] * 0.000001
			);

			destLatitude = receivedLatitude;
			destLongitude = receivedLongitude;
		}
	}
}

ISR(USART3_RX_vect){
	// Parse data
    received_data = UDR3;
    if (received_data == '\n') {
        // Parse formed line
        if(strstr(received_string, "GPR")){
            // printf("%s\n", received_string);
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, received_string)) {
				latitude = minmea_tocoord(&frame.latitude);
				longitude = minmea_tocoord(&frame.longitude);
				speed = minmea_tofloat(&frame.speed);
				boatCoordinateReceived = 1;
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
}

void USART_initGPS(void){
    UBRR3H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR3L = (uint8_t)(BAUD_PRESCALER);
    UCSR3B = (1<<RXEN3)|(1<<TXEN3);
    UCSR3C = ((1<<UCSZ30)|(1<<UCSZ31));
}

void USART_initRF(void){
	UBRR2H = (unsigned char)(BAUD_PRESCALER >> 8);
    UBRR2L = (unsigned char) BAUD_PRESCALER;
    UCSR2B = (1 << RXEN2) | (1 << TXEN2);
    UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);	
}

void Timer1_Init() {
    // Set non-inverting mode for OC1A
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
    // Set WGM13 to enable PWM mode with ICR1 as top
    TCCR1B |= (1 << WGM13);
    // Set prescaler to 8 and start the timer1
    TCCR1B |= (1 << CS11);
    // Set ICR1 value for 200Hz PWM frequency
    ICR1 = 9999; // (16MHz / 8) / 200Hz - 1 = 9999
}

void Set_Thrust(int pulse_width, int pulse_width2) { // right, then left motor
    OCR1A = pulse_width;
    OCR1B = pulse_width2;
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

int getPWMReduction(float deltaAngle){
	float deltaPWM = (deltaAngle / 180.0) * (MAX_FORWARD - CUTOFF);
	return ((int) deltaPWM);
}

void Calibrate_Magnetometer(){
	int startValueX;
	int startValueY;
	int neutralValueX;
	int neutralValueY;
	int endValueX;
	int endValueY;

	OCR3A = START; // Set position to 0 degrees
	_delay_ms(1000);
	for(int i = 0; i < 1; i++){
		startValueX = Magneto_GetX();
		startValueY = Magneto_GetY();
		_delay_ms(500);
		OCR3A = NEUTRAL; // Set position to 90 degrees
		_delay_ms(1000);

		neutralValueX = Magneto_GetX();
		neutralValueY = Magneto_GetY();
		_delay_ms(500);
		OCR3A = END; // Set position to 180 degrees (not neccessary)
		_delay_ms(1000);

		endValueX = Magneto_GetX();
		endValueY = Magneto_GetY();
		_delay_ms(500);
		OCR3A = START;
		_delay_ms(1000);
	}

	_delay_ms(1000);

	int deltaX = endValueX - startValueX;
	int deltaY = endValueY - startValueY;
	offsetX = -1 * (startValueX + (deltaX / 2));
	offsetY = -1 * (startValueY + (deltaY / 2));
}



