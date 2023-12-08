#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"

#include <avr/interrupt.h>

int volatile counter_ms;

int main() {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
    // 2 step process to enable the LCD
    // initialize the 12c communication, LEDs
    i2c_init(); 
    // initialize the LCD 
    LCD_init(); 
    // initialize the temperature sensor
    lm75_init(); 
    // configuration of the LEDs
    DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    PORTD = 0x00; // Set output LEDs to off

    // configuration of the input buttons
    DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    PORTC = 0x3F; // Enable internal pull at PC0..3 inputs

    // Enable CTC mode
    TCCR0A |= (1<<WGM01);
    // Set the number of ticks that I want to count to
    OCR0A = 249; // in hex 0xf9, we have 1ms
    // OCR0A = 124; // so we have half ms
    // Start the timer and set prescaler to 64
    //TCCR0B |= (1<<CS01)|(1<<CS00);
    // Start the timer and set the prescaler to no prescaling, prescaler of one
    //TCCR0B |= (1<<CS00);
    // Start the timer and set prescaler to 16*64
    TCCR0B |= (1<<CS02)|(1<<CS00);
    //if I want to use timer 2 not 0 I change 0 to 2 in the for example TCCR2B
    TIMSK0 |= (1<<OCIE0A); // subscribe  to intterupts when timer 0 matches A
    sei();


    // set the PC1 as input to trigger an LED to be on for 1 second
    // PC1 -> PCINT9, belongs to group1
    PCICR |= (1<<PCIE1);
    // enable interrupts from PCINT9 as part of group1
    PCMSK1 |= (1<<PCINT9);
    while(1)
    {

        
        while((TIFR0 & (1<<OCF0A))==0) //wait for the overflow event
        {

        } 
        // 1ms have passed
        PORTD ^= 0xF0; // toggle all leds (PD4...PD7)
        // reset the overflow flag
        TIFR0 = (1<< OCF0A);



    }
    
   
}

ISR(TIMER0_COMPA_vect){
    //PORTD ^= 0xF0;//toggle all leds (PD4...PD5)
    if (counter_ms>0){
        counter_ms--;
    }
    if(counter_ms==0){
        PORTD = 0x00; //turn all LEDs off
    }
}

ISR(PCINT1_vect){

    PORTD ^= 0xFF;

    if((PINC & (1<<PINC1)) == 1<<PINC1)
    {
        PORTD = 0xFF;// turn on all LEDs
        counter_ms = 500; // start from 500
    }
    else{
        /* HIGH to LOW pin change */
    }
}