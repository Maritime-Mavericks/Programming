#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
//#include "lcd.h"
//#include "lm75.h"
#include <avr/interrupt.h>

volatile int count=0;
void init_interrupt();//function with interrupts

int main() {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
    // 2 step process to enable the LCD
    // initialize the 12c communication, LEDs
    // i2c_init(); 
    // initialize the LCD 
    // //LCD_init(); 
    // initialize the temperature sensor
    // lm75_init(); 
    // configuration of the LEDs
    DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    PORTD = 0x00; // Set output LEDs to off

    // // configuration of the input buttons
    // DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    // PORTC = 0x3F; // Enable internal pull at PC0..3 inputs
    
    init_interrupt();

    
   
    while(1){
        //do nothing
    }

}

ISR (TIMER0_COMPA_vect)  // timer0 overflow interrupt
{
    //event to be executed every 1ms here
    count ++; //a variable to count the number of interrupts occurred
    if(count == 500){
    PORTD ^= (1<<4);// (1 na 4 miejsce), compare PORTD the one that is now with the LED 1 on
    count = 0;
    }
}
void init_interrupt(){
    // Set the Timer Mode to CTC
    TCCR0A |= (1 << WGM01);
    // Set the value that you want to count to
    OCR0A = 0xF9;//1ms
    TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect
    sei();         //enable interrupts
    TCCR0B |= (1 << CS01) | (1 << CS00);
    // set prescalerto 64 and start the timer
}