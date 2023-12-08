#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include <avr/eeprom.h>

unsigned int address_max = 8;
unsigned int address_min = 1;


void printing( uint8_t, uint8_t, uint8_t);//uint8_t lets us print to eeprom
int main() {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
     // 2 step process to enable the LCD
     // initialize the 12c communication 
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
    
    uint8_t current_temperature = get_temperature();
  
    
    uint8_t min=current_temperature;
    uint8_t max=current_temperature;

    max = eeprom_read_byte((uint8_t *)address_max); // read a byte stored at the address 0 in the eeprom memory'
   
    min = eeprom_read_byte((uint8_t *)address_min); // read a byte stored at the address 1 in the eeprom memory
    
    
   while(1){
    current_temperature = get_temperature();//taking temperature from the temperature sensor
    
    if(current_temperature>max){
        max=current_temperature;
        eeprom_write_byte((uint8_t *)address_max, (uint8_t)max); 
    }
    if(current_temperature<min){
        min=current_temperature;
        eeprom_write_byte((uint8_t *)address_min, (uint8_t)min); 
    }
    printing(current_temperature, max, min);
    if(PINC == 0b00110111){//if button 4 is pressed
        min=100;
        max=0;
        eeprom_write_byte((uint8_t *)address_max, (uint8_t)max); 
        eeprom_write_byte((uint8_t *)address_min, (uint8_t)min); 
    }
    _delay_ms(1000);
    
   }
}

void printing(uint8_t current_temperature, uint8_t max, uint8_t min){
    LCD_clear();
    LCD_set_cursor(0, 0);
    printf("Current temp: %d", current_temperature);
    LCD_set_cursor(0, 1);
    printf("Max temp: %d  ", max);
    LCD_set_cursor(0, 2);
    printf("Min temp: %d  ", min);
    
    
}