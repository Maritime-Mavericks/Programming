#include <avr/io.h>
#include <util/delay.h>

#define START 999
#define NEUTRAL 2999
#define END 4999

int main(void){

	uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

	// Initialize I2C communication
	i2c_init();

	DDRB |= (1<<PB1); // Set PB1 as output

	TCCR1A |= (1<<COM1A1) | (1<<WGM11); // Fast PWM, non-inverting mode
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Fast PWM, prescaler = 8
	ICR1=39999;   //20ms PWM period
	//OCR1A = 1999; // Set initial position to 90 degrees

	OCR1A = 999; // Set position to 0 degrees
	_delay_ms(2000);

	for(int i = 0; i < 2; i++){
		for(int angle = 0; angle <= 180; angle = angle + 5){
			printf("%d\n", angle);
			OCR1A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
		for(int angle = 180; angle >= 0; angle = angle - 5){
			printf("%d\n", angle);
			OCR1A = START + ((((float) angle) / 180) * (END - START));
			_delay_ms(200);
		}
	}

	_delay_ms(1000);

	OCR1A = 999; // Set position to 0 degrees
	_delay_ms(1000);

	return 0;
}