#include <avr/io.h>
#include <stdio.h>
#include <usart.h>
#include <util/delay.h>
#include <i2cmaster.h>

#define START 999
#define NEUTRAL 2999
#define END 4999

int main(void){

	uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

	// Initialize I2C communication
	i2c_init();

	DDRE |= (1<<PE3); // Set PB5 as output

	TCCR3A |= (1<<COM3A1) | (1<<WGM31); // Fast PWM, non-inverting mode
	TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31); // Fast PWM, prescaler = 8
	ICR3=39999;   //20ms PWM period
	//OCR1A = 1999; // Set initial position to 90 degrees

	OCR3A = 999; // Set position to 0 degrees
	_delay_ms(2000);

	for(int i = 0; i < 2; i++){
		for(int angle = 0; angle <= 180; angle = angle + 5){
			printf("%d\n", angle);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
		for(int angle = 180; angle >= 0; angle = angle - 5){
			printf("%d\n", angle);
			OCR3A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
	}

	_delay_ms(1000);

	OCR3A = 999; // Set position to 0 degrees
	_delay_ms(1000);

	return 0;
}