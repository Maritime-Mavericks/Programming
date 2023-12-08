#define F_CPU 16000000UL
#include <stdio.h>
//#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
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
    UCSR0B|= (1<<RXCIE0);//enable interrupts for RXIE
    sei(); //enable interrupts
    // configuration of the LEDs
    DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    PORTD = 0x00; // Set output LEDs to off

    // configuration of the input buttons
    DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    PORTC = 0x3F; // Enable internal pull at PC0..3 inputs

    
   
    while(1){        
        //Infinite loop
    }

return 0;
} 

void usart_init(void){
    UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALER);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
    }

void usart_send( unsigned char data){
    while(!(UCSR0A & (1<<UDRE0))); //wait for transmit buffer
    UDR0 = data; //data to be sent
}

unsigned char usart_receive(void){
    while(!(UCSR0A & (1<<RXC0))); //wait for new data
    return UDR0; //received data
}

ISR(USART_RX_vect){
volatile unsigned char received_data = UDR0;
if (received_data== 1) PORTD = 0b00010000; //led1 on
if (received_data== 2) PORTD = 0b00100000; //led2 on
if (received_data== 3) PORTD = 0b01000000; //led3 on
if (received_data== 4) PORTD = 0b10000000; //led4 on
}