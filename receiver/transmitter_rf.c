#define F_CPU 16000000UL
#include <stdio.h>
//#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL)))-1)
//Function prototypes
void usart_init(void);
unsigned char usart_receive(void);
int result=0;

int main(void){
    usart_init();        //Call the USART initialization code
    UCSR0B|= (1<<RXCIE0);//enable interrupts for RXIE
    sei(); //enable interrupts
    // configuration of the LEDs
    //DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    //PORTD = 0x00; // Set output LEDs to off
    DDRB = (0<<5); // set bit 5 at ddrb low as output, so it is low from the begginning(PB5)
    // configuration of the input buttons
    DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    //PORTC = 0x3F; // Enable internal pull at PC0..3 inputs
    PORTC |= (0<<PC0);
    
   
    while(1){     
       _delay_ms(200);   
       usart_send(1);  
    //    _delay_ms(5000); 
    //   usart_send(2);
       
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

