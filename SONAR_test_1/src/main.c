#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>

#include "usart.h"

float get_distance(float, float);
volatile int detection = 0;

int main(void) {    
    
    DDRD  = 0xFF; // I/O board:PD4...7 as outputs, for LEDs
    DDRC  = 0xF0; // I/O board PC0...3 as inputs, for buttons
    PORTC = 0x3F; // Enable internal pull at PC0..3 inputs
    PORTD = 0x00; // Set output LEDs to off
    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

    // i2c_init();
    // lm75_init();
    // LCD_init();
    // get_temperature();
    // LCD_set_cursor(0, 0);

    DDRD |= (1 << DDD3); // Set INT1 to be output (trigger)

    DDRD &= ~(1 << DDD2); // Clear the PD2 pin, PD2 (INT0 pin) is now an input
    PORTD |= (1 << PORTD2); // turn On the Pull-up
    // PD2 is now an input with pull-up enabled (MIGHT NOT BE NEEDED)
    EICRA |= (1 << ISC01); // set INT0 to trigger on falling edge
    EIMSK |= (1 << INT0); // Turns on INT0
    sei();

    // Setup timer
    TCCR0A |= (1 << WGM01);
    OCR0A = 0x13; // equivalent of delay which is 0.01 millisecond (using formula)
    TCCR0B |= (1 << CS01); // 8 prescaler

    while(1){
        float milliseconds = 0;
        detection = 0;
    
        // Send wave
        PORTD |= (1 << PORTD3); // Set TRIG to high
        _delay_us(10);
        PORTD &= ~(1 << PORTD3); // Set TRIG to low

        // Wait for wave to come back
        while(1){
            // wait for overflow to occur
            while((TIFR0 & (1 << OCF0A)) == 0){
                
            }
            // Reset to 0
            TIFR0 |= (1 << OCF0A); 
            milliseconds += 0.01;


            if(detection == 1){
                break;
            }
        }

        // Calculate time
        printf("DeltaT (ms): %f\n", milliseconds);

        // Print distance
        float dist = get_distance(345.0, milliseconds);
        float distInCM = dist * 100;
        printf("Distance: %f\n\n", distInCM);

        // Wait for a second
        _delay_ms(1000);
    }

    return 0;
}

float get_distance(float speed, float ms){
    return ((speed * ms * 0.001) / 2);
}

ISR (INT0_vect){ // external interrupt 0
    //event to be executed when a logic change / edge on INT0 pin occurs
    detection = 1;
}



