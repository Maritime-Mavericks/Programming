#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"

void delay_ms(unsigned int milliseconds);
void delay_hs(unsigned int hundred_milliseconds);

int main() {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication 
    // initialize the 12c communication, LEDs
    i2c_init(); 
    // configuration of the LEDs
    
    
    // DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    // PORTD = 0x00; // Set output LEDs to off

    DDRD &= ~(1 << DDD5);   
    // Clear the PD4 pin
    // PD4 is now an input
    PORTD |= (1 << PORTD5);   
    // turn On the Pull-up
    // PD4 is now an input with pull-up enabled
    TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);
    // Turn on the counter, Clock on Rise

    while(1){
       //task 5a
        // PORTD = 0b00010000; //LEDs on
        // delay_ms(100); // 0.1 second delay
        // PORTD = 0b00000000; //LEDs off

        // delay_hs(20);// 2 second delay

        // PORTD = 0b00010000; //LEDs on
        // delay_hs(5); // 0.5 second delay
        // PORTD = 0b00000000; //LEDs off

        // delay_hs(20);// 2 second delay

        // PORTD = 0b00010000; //LEDs on
        // delay_hs(10); // 1 second delay
        // PORTD = 0b00000000; //LEDs off

        // delay_hs(20);// 2 second delay
       
        // PORTD = 0b00010000; //LEDs on
        // delay_hs(20); // 2 second delay
        // PORTD = 0b00000000; //LEDs off
    
        // delay_hs(20);// 2 second delay
        
        
       //task 5b
        printf("value %d\n", TCNT1);
        delay_ms(1000); // 1 second delay
       

    }
   
}
void delay_ms(unsigned int milliseconds){
        for(int i=0; i<milliseconds; i++){
        // Set the Timer Mode to CTC
        TCCR0A |= (1 << WGM01);
        // Set the value that you want to count to
        OCR0A = 0xF9;
        // start the timer
        TCCR0B |= (1 << CS01) | (1 << CS00);
        // set prescalerto 64 and start the timer
        while ( (TIFR0 & (1 << OCF0A) ) == 0)  // wait for the overflow event
        {
        }

        // reset the overflow flag
        TIFR0 = (1 << OCF0A);
        }
}
void delay_hs(unsigned int hundred_milliseconds){
        for(int i=0; i<(hundred_milliseconds*100); i++){
        // Set the Timer Mode to CTC
        TCCR0A |= (1 << WGM01);
        // Set the value that you want to count to
        OCR0A = 0xF9;
        // start the timer
        TCCR0B |= (1 << CS01) | (1 << CS00);
        // set prescalerto 64 and start the timer
        while ( (TIFR0 & (1 << OCF0A) ) == 0)  // wait for the overflow event
        {
        }

        // reset the overflow flag
        TIFR0 = (1 << OCF0A);
        }
}