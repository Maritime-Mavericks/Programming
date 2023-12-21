#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "usart.h"
#include "i2cmaster.h"

// from the controller datasheet 
#define MAX_FORWARD 1600// pwm max cw thrust (1900us)
#define MAX_REVERSE 1400 //pwm max ccw (1100us)
#define STOP 1500 // pwm stop(1500us)

// Initialize Timer1 to generate PWM at 200Hz (5ms period)
// the datashhet says max 400HZ, i put it ro half
void Timer4_Init() {
    // Set non-inverting mode for OC1A
    TCCR4A |= (1 << COM4A1) | (1 << COM4B1);
    // Set WGM13 to enable PWM mode with ICR1 as top
    TCCR4B |= (1 << WGM43);
    // Set prescaler to 8 and start the timer1
    TCCR4B |= (1 << CS41);
    // Set ICR1 value for 200Hz PWM frequency
    ICR4 = 9999; // (16MHz / 8) / 200Hz - 1 = 9999
}



// pwm
void Set_Thrust(int pulse_width,int pulse_width2) {
    OCR4A = pulse_width;
    OCR4B = pulse_width2;
}


int main(void) {

    uart_init();
    io_redirect();

    DDRH |= (1 << PH3); // D6 (left)
    DDRH |= (1 << PH4); // D7 (right)

    printf("Done setting output \n");
    
    Timer4_Init();
    printf("Done timer 4 initialize \n"); 
    //Initialize
    Set_Thrust(STOP,STOP); 
    printf("Stopped both motors \n"); 
    _delay_ms(1000); 
    // Run the motor forward for 5 seconds
    _delay_ms(3000);
    _delay_ms(300); 
    Set_Thrust(MAX_REVERSE, MAX_FORWARD);
    _delay_ms(3000);
    Set_Thrust(STOP,STOP);
   
    while(1) {
      
        break;
    }
    
}
