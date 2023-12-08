#define F_CPU 16000000UL

#include <stdio.h>// + delay, + usart, + io
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"

#define JOYSTICK_X_PIN PC4
#define JOYSTICK_Y_PIN PC5

void initialize_adc() {
    ADMUX |= ADMUX | 0x40;                      // selects Avcc as the reference voltage
    ADMUX |= (1 << REFS0);                     // Reference voltage on AVCC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC clock prescaler /128
    ADCSRA |= (1 << ADEN);                     // Enable ADC
}

uint16_t read_adc_x() {
    ADMUX = (ADMUX & 0xF0) | (JOYSTICK_X_PIN & 0xF0); // Select ADC channel for X-axis
    ADCSRA |= (1 << ADSC);                            // Start ADC conversion
    while (ADCSRA & (1 << ADSC));                      // Wait for conversion to complete
    return ADC;                                       // Return ADC value
}

uint16_t read_adc_y() {
    ADMUX = (ADMUX & 0xF0) | (JOYSTICK_Y_PIN & 0xF0); // Select ADC channel for Y-axis
    ADCSRA |= (1 << ADSC);                            // Start ADC conversion
    while (ADCSRA & (1 << ADSC));                      // Wait for conversion to complete
    return ADC;                                       // Return ADC value
}
// int button1 = 0b00111110;
// int button2 = 0b00111101;
// int button3 = 0b00111011;
// int button4 = 0b00110111;

// int LED1 = 0b00010000;
// int LED2 = 0b00100000;
// int LED3 = 0b01000000;
// int LED4 = 0b10000000;


int main() {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
    // 2 step process to enable the LCD
    // initialize the 12c communication 
    i2c_init(); 
    // initialize the LCD 
    //LCD_init(); 
    // initialize the temperature sensor
    //lm75_init(); 
    // configuration of the LEDs
    // DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    // PORTD = 0x00; // Set output LEDs to off

    // configuration of the input buttons
   // DDRC = 0b11110000; //I/O board PC0…3 as inputs, for buttons
    DDRC &= ~(1<<PC0);
    DDRC &= ~(1<<PC1);
    DDRC &= ~(1<<PC2);
    DDRC &= ~(1<<PC3);


    PORTC |= (1<<PC0);
    PORTC |= (1<<PC1);
    PORTC |= (1<<PC2);
    PORTC |= (1<<PC3);

   
    
   
   while(1)
    {
    if(PINC == 0b00111110){
        printf("but 1\n");
    }
    if(PINC == 0b00111101){
        printf("but 2\n");
    }
    if(PINC == 0b00111011){
        printf("but 3\n");
    }
    if(PINC == 0b00110111){
        printf("but 4\n");
    }
    
    
    }
return 0;
}
