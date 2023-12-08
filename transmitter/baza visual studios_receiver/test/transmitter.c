#define F_CPU 16000000UL
#include <stdio.h>
//#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>
#include "i2cmaster.h"
#define BAUDRATE 57600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL)))-1)
//Function prototypes
void usart_init(void);
unsigned char usart_receive(void);
void usart_send(unsigned char data);

int BTN1 = 0x3E;
int BTN2 = 0x3D;
int BTN3 = 0x3B;
int BTN4 = 0x37;



int main(void){
    usart_init();        //Call the USART initialization code
    
    // configuration of the LEDs
    DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    PORTD = 0x00; // Set output LEDs to off

    // configuration of the input buttons
    DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    PORTC = 0x3F; // Enable internal pull at PC0..3 inputs

    

    while(1){        //Infinite loop

    if (PINC == BTN1) { //button 1
        usart_send(1);  
       
        _delay_ms(500);
    }
    if (PINC == BTN2) {    //button 2
      
        usart_send(2);  
        _delay_ms(500);
    }
     if (PINC == BTN3) {   //button 3
        usart_send(3);  
    
        _delay_ms(500);
    }
     if (PINC == BTN4) {   //button 4
        usart_send(4);  
    
        _delay_ms(500);
    }
  
    }

return 0;
} 

void usart_init(void){
    UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALER);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
    }

void usart_send(unsigned char data){
    while(!(UCSR0A & (1<<UDRE0))); //wait for transmit buffer
    UDR0 = data; //data to be sent
}


