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
#include "MPU6050_res_define.h"

#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#define START 999
#define NEUTRAL 2999
#define END 4999

#define PI	3.14159265359	/* Define Pi value */
#define Declination	0.0527/* Define declination of location from where measurement going to be done */
#define RAD_TO_DEG (180.0 / PI)

#define MAX_FORWARD 1300// pwm max cw thrust = 1900us
#define MAX_REVERSE 1700 //pwm max ccw = 1100us
#define STOP 1500 // pwm stop(1500us)
#define CUTOFF_FORWARD 1450 // forward cutoff for proportional controller
#define CUTOFF_REVERSE 1550 // reverse cutoff for proportional controller

volatile char received_data;
char received_string[512]; // Maximum string length, adjust as needed
volatile uint8_t string_index = 0;
volatile int boatCoordinateReceived = 0;
volatile float latitude;
volatile float longitude;
volatile float speed;
float initialLatitude = 0;
float initialLongitude = 0;

volatile int startRescueTransmission = 0;
volatile int goingBackAfterPickUp = 0;
volatile int digitNumber = 0;
volatile int latOrLong = 0; // 0 is undefined, 17 is latitude, 31 is longitude
volatile int latDigits[8];
volatile int longDigits[7];
float destLatitude = 0;
float destLongitude = 0;
int firstDestinationCoordReceived = 0;
float dist = 0;

int offsetX = 0;
int offsetY = 0;

int timeForOneIteration = 20;
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

int stopProgramAndMotors = 0;

void USART_initRF(void);
void USART_initGPS(void);
void Timer4_Motors_Init(void);
void Set_Thrust(int pulse_width, int pulse_width2); // right then left motor
void Magneto_init(void);
float Magneto_GetHeading(void);
float Magneto_GetHeadingOffset(void);
int Magneto_GetX(void);
int Magneto_GetY(void);
int Magneto_GetZ(void);
void PWM_Servo_Init(void);
void Calibrate_Magnetometer(void);
int getPWMReduction(float deltaAngle);
void Gyro_Init(void);
void MPU_Start_Loc(void);
void Read_RawValue(void);
void Timer0_init(void);
void Calibrate_MPU6050();
void Calculate_Kalman_Filter(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, int milliseconds);
void Calculate_Acc_Gyr_Kalman_Angles(void);
void Stop_motors(void);
float getDestHeading(float currentLatitude, float currentLongitude, float currentDestLatitude, float currentDestLongitude);
void calculate_deltaAngle_And_Turn(float destHeading, float heading, int *turnRightOrLeft, float *deltaAngle);
float computeLatitude();
float computeLongitude();
float getCompensatedHeading();
float Magneto_calculateHeading(float x, float y);

int main(void){

	uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication
	i2c_init(); // Initialize I2C communication
	
	Magneto_init();	
	PWM_Servo_Init();

	USART_initGPS(); 
	USART_initRF();

	Timer4_Motors_Init(); 

	Gyro_Init();
	Timer0_init();

	sei();

	printf("\nStopping motors soon...\n");
    Stop_motors();
	_delay_ms(1000);

	printf("\nCalibrating magnetometer...\n");
	Calibrate_Magnetometer();

	_delay_ms(1000);
	
	printf("\nCalibrating accelerometer and gyroscope...\n\n");
	Calibrate_MPU6050();

    sei(); //enable interrupts

	while(1){ // empty infinite loop
		if(counter >= timeForOneIteration){ // in milliseconds
			Calculate_Acc_Gyr_Kalman_Angles();
		}

		if(stopProgramAndMotors == 1){
			break;
		}

		if(startRescueTransmission == 1 && firstDestinationCoordReceived == 1){
			if(boatCoordinateReceived == 1){
				if(goingBackAfterPickUp == 1){
					destLatitude = initialLatitude;
					destLongitude = initialLongitude;
				}

				// Calculate angles
				float biasAngle = 0;
				float headingCompensated = getCompensatedHeading() + biasAngle;
				float destHeading = getDestHeading(latitude, longitude, destLatitude, destLongitude);

				// Find direction to rotate
				int turnRightOrLeft = 0; // 0 is right, left is 1
				float deltaAngle = 0;
				calculate_deltaAngle_And_Turn(destHeading, headingCompensated, &turnRightOrLeft, &deltaAngle);

				// Find distance to target
				float y = (destLatitude - latitude) * (2 * PI * 6371) / 360.0;
				float x = (destLongitude - longitude) * (2 * PI * 6371) / 360.0;
				dist = sqrt(pow(x, 2) + pow(y, 2)) * 1000.0;

				float heading = Magneto_GetHeadingOffset();

				printf("Lat: %f    Long: %f\n", latitude, longitude);
				printf("DestLat: %f    DestLong: %f\n", destLatitude, destLongitude);
				printf("heading: %0.0f, destHead: %0.0f\n", headingCompensated, destHeading);
				printf("KalmanX: %0.3f, KalmanY: %0.3f\n", KalmanAngleX, KalmanAngleY);
				printf("Heading (uncompensated): %0.0f\n", heading);
				printf("Distance: %0.1f\n", dist);

				int pwmReduction = getPWMReduction(deltaAngle);
				if(turnRightOrLeft == 0){ // If turn right, make right motor lesser. Set left to max.
					printf("Turn right\n");
					Set_Thrust(MAX_REVERSE + pwmReduction, MAX_FORWARD);
				} else { // If turn left, make left motor lesser. Set right to max.
					printf("Turn left\n");
					Set_Thrust(MAX_REVERSE, MAX_FORWARD - pwmReduction);
				}
				
				// Check if boat has reached destination coords
				if(dist <= 5.0){
					if(goingBackAfterPickUp == 0){ // going to pickup, reached person
						printf("Going back now...\n");

						// Implement button press, motor stop wait

						goingBackAfterPickUp = 1;
					} else if(goingBackAfterPickUp == 1){ // going back, reached shore
						printf("Finished rescue mission...\n");
						
						// Implement stop motors and everything

						goingBackAfterPickUp = 0;
						startRescueTransmission = 0;
						break;
					}
				}

				printf("\n");

				boatCoordinateReceived = 0;
			}
		}
    }

	Stop_motors();
	printf("Exiting program...\n");

	return 0;
}

ISR(USART2_RX_vect){
    received_data = UDR2;
    int digit = received_data - 48;

	if(digit == 35 && startRescueTransmission == 0){ // start
		startRescueTransmission = 1;
		// Set initial coords
		initialLatitude = latitude;
		initialLongitude = longitude;
	}

	if(digit == 32){ // STOP/PAUSE
		stopProgramAndMotors = 1;
	}

	if(startRescueTransmission == 1 && goingBackAfterPickUp == 0){
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
			destLatitude = computeLatitude();
			destLongitude = computeLongitude();

			firstDestinationCoordReceived = 1;
		}
	}
}

ISR(USART3_RX_vect){
	// Parse data
    received_data = UDR3;
    if (received_data == '\n') {
        // Parse formed line
        if(strstr(received_string, "GPR")){
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

ISR(TIMER0_COMPA_vect) {
    counter++;
	counter2++;
}

float getCompensatedHeading(){
	int x = Magneto_GetX() + offsetX;
	int y = Magneto_GetY() + offsetY;
	int z = Magneto_GetZ();

	float xAccAngle = KalmanAngleX / RAD_TO_DEG;
	float yAccAngle = -1 * KalmanAngleY / RAD_TO_DEG;

	float xComp = x*cos(yAccAngle) + 
					  y*sin(xAccAngle)*sin(yAccAngle) -
					  z*cos(xAccAngle)*sin(yAccAngle);
	float yComp = y*cos(xAccAngle) -
					  z*sin(xAccAngle);

	float compHeading = Magneto_calculateHeading(xComp, yComp);

	return compHeading;
}

float Magneto_calculateHeading(float x, float y){
	float ratio = (y / x);
	float angle = atan(ratio) * RAD_TO_DEG;
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

float computeLatitude(){
	float calculatedLat = (
		latDigits[0] * 10 +
		latDigits[1] * 1 +
		latDigits[2] * 0.1 +
		latDigits[3] * 0.01 +
		latDigits[4] * 0.001 +
		latDigits[5] * 0.0001 +
		latDigits[6] * 0.00001 + 
		latDigits[7] * 0.000001
	);
	return calculatedLat;
}

float computeLongitude(){
	float calculatedLong = (
		longDigits[0] * 1 +
		longDigits[1] * 0.1 +
		longDigits[2] * 0.01 +
		longDigits[3] * 0.001 +
		longDigits[4] * 0.0001 +
		longDigits[5] * 0.00001 +
		longDigits[6] * 0.000001
	);
	return calculatedLong;
}

void calculate_deltaAngle_And_Turn(float destHeading, float heading, int *turnRightOrLeft, float *deltaAngle){
	float destHeadMinusHead = destHeading - heading;
	float headMinusDestHead = heading - destHeading;

	if(destHeadMinusHead >= 0){ // CASE 1 
		if(destHeadMinusHead >= 180){ 
			// turn right
			*turnRightOrLeft = 0;
			*deltaAngle = (360 - destHeadMinusHead);
		} else {
			// turn left
			*turnRightOrLeft = 1;
			*deltaAngle = destHeadMinusHead;
		}
	} else { // CASE 2
		if(headMinusDestHead >= 180){ 
			// turn left
			*turnRightOrLeft = 1;
			*deltaAngle = (360 - headMinusDestHead);
		} else {
			// turn right
			*turnRightOrLeft = 0;
			*deltaAngle = headMinusDestHead;
		}
	}
}

float getDestHeading(float currentLatitude, float currentLongitude, float currentDestLatitude, float currentDestLongitude){
	float deltaLat = currentDestLatitude - currentLatitude; // y (kind of)
	float deltaLong = currentDestLongitude - currentLongitude; // x (kind of)
	float ratio = ((float)deltaLat ) / ((float)deltaLong);
	float angle = fabs(atan(ratio) * 180.0 / ((float) PI));
	float destHeading = 0;
	if(deltaLong <= 0 && deltaLat >= 0){
		destHeading = 90.0 - angle;
	} else if(deltaLong <= 0 && deltaLat <= 0){
		destHeading = 90.0 + angle;
	} else if(deltaLong >= 0 && deltaLat <= 0){
		destHeading = 270.0 - angle;
	} else if(deltaLong >= 0 && deltaLat >= 0){
		destHeading = 270.0 + angle;
	}
	return destHeading;
}

void Timer0_init(){
	TCCR0A |= (1 << WGM01);
    OCR0A = 0xF9; // equivalent of delay which is 1 millisecond (using formula)
    TCCR0B |= (1 << CS01) | (1 << CS00); // 64 prescaler
    TCNT0 = 0;
	TIMSK0 |= (1 << OCIE0A);
}

void Calculate_Acc_Gyr_Kalman_Angles(){
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

	Calculate_Kalman_Filter(KalmanAngleX, KalmanAngleXUncertainty, Xg, xAngleAcc, timeForOneIteration);
	KalmanAngleX = KalmanAnglePrediction;
	KalmanAngleXUncertainty = KalmanUncertaintyPrediction;
	Calculate_Kalman_Filter(KalmanAngleY, KalmanAngleYUncertainty, Yg, yAngleAcc, timeForOneIteration);
	KalmanAngleY = KalmanAnglePrediction;
	KalmanAngleYUncertainty = KalmanUncertaintyPrediction;
	
	float alpha = 0.15;
	ComplimentaryAngleX = alpha * xAngleGyr + (1 - alpha) * xAngleAcc;
	ComplimentaryAngleY = alpha * yAngleAcc + (1 - alpha) * yAngleAcc;

	counter = 0;
}

void Calculate_Kalman_Filter(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, int milliseconds){
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

void Stop_motors(){
	Set_Thrust(STOP,STOP); 
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

void USART_initGPS(void){
    UBRR3H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR3L = (uint8_t)(BAUD_PRESCALER);
    UCSR3B = (1<<RXEN3)|(1<<TXEN3);
    UCSR3C = ((1<<UCSZ30)|(1<<UCSZ31));

	UCSR3B |= (1<<RXCIE3); //enable interrupts for RXIE for GPS
}

void USART_initRF(void){
	UBRR2H = (unsigned char)(BAUD_PRESCALER >> 8);
    UBRR2L = (unsigned char) BAUD_PRESCALER;
    UCSR2B = (1 << RXEN2) | (1 << TXEN2);
    UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);	

	UCSR2B |= (1<<RXCIE2); //enable interrupts for RXIE for RF
}

void Timer4_Motors_Init() {
	DDRH |= (1 << PH4); // D7 (right), OC4B
	DDRH |= (1 << PH3); // D6 (left), OC4A

    // Set non-inverting mode for OC4A
    TCCR4A |= (1 << COM4A1) | (1 << COM4B1);
    // Set WGM43 to enable PWM mode with ICR4 as top
    TCCR4B |= (1 << WGM43);
    // Set prescaler to 8 and start the timer4
    TCCR4B |= (1 << CS41);
    // Set ICR4 value for 200Hz PWM frequency
    ICR4 = 9999; // (16MHz / 8) / 200Hz - 1 = 9999
}

void Set_Thrust(int pulse_width, int pulse_width2) { // right, then left motor
    OCR4B = pulse_width; // right
    OCR4A = pulse_width2; // left
}

void PWM_Servo_Init(){
	DDRE |= (1<<PE3); 
	TCCR3A |= (1<<COM3A1) | (1<<WGM31); // Fast PWM, non-inverting mode
	TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31); // Fast PWM, prescaler = 8
	ICR3 = 39999;   //20ms PWM period
}

void Magneto_init(){		/* Magneto initialize function */
	i2c_start(0x3C);	/* Start and write SLA+W */
	i2c_write(0x00);	/* Write memory location address */
	i2c_write(0x70);	/* Configure register A as 8-average, 15 Hz default, normal measurement */
	i2c_write(0xA0);	/* Configure register B for gain */
	i2c_write(0x00);	/* Configure continuous measurement mode in mode register */
	i2c_stop();		/* Stop I2C */
}

float Magneto_GetHeading(){
	int x = Magneto_GetX();
	int y = Magneto_GetY();
	float ratio = ((float)y ) / ((float)x);
	float angle = fabs(atan(ratio) * 180.0 / ((float) PI));
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

float Magneto_GetHeadingOffset(){
	int x = Magneto_GetX() + offsetX;
	int y = Magneto_GetY() + offsetY;
	float ratio = ((float)y ) / ((float)x);
	float angle = fabs(atan(ratio) * 180.0 / ((float) PI));
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
	float deltaPWM = (deltaAngle / 180.0) * (MAX_FORWARD - CUTOFF_FORWARD); // should be the same with (CUTOFF_REVERSE - MAX_REVERSE)
	return ((int) deltaPWM);
}

void Calibrate_Magnetometer(){
	int startValueX;
	int startValueY;
	int endValueX;
	int endValueY;

	OCR3A = START; // Set position to 0 degrees
	_delay_ms(1000);
	for(int i = 0; i < 1; i++){
		startValueX = Magneto_GetX();
		startValueY = Magneto_GetY();
		_delay_ms(500);
		OCR3A = END;
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



