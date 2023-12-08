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
void usart_init();
unsigned char usart_receive(void);
//void usart_send(unsigned char data);
int result=0;
volatile unsigned char received_data;

ISR(USART_RX_vect){
received_data = UDR0;
 //led on
}


int main(void){
    usart_init();        //Call the USART initialization code
    //io_redirect(); // redirect input and output to the communication
    UCSR0B|= (1<<RXCIE0);//enable interrupts for RXIE
    sei(); //enable interrupts
    // configuration of the LEDs
    DDRD = 0x00; // I/O board: PD4...7 as output, for LEDs
    PORTD = 0xFF; // Set output LEDs to off
    DDRB = (0<<5); // set bit 5 at ddrb low as output, so it is low from the begginning(PB5)
    // configuration of the input buttons
    DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    //PORTC = 0x3F; // Enable internal pull at PC0..3 inputs
    PORTC |= (0<<PC0);

    PORTB &= ~(1<<5);
    

    DDRB= 0xff;
   
    
    
    while(1){ 
    if(usart_receive()==1){
        PORTB|=(1<<5);
       _delay_ms(100);
    }
    // if(usart_receive()==2){
    //     PORTB|=(0<<5);
    //     //_delay_ms(100);
    // }
    
    }

return 0;
} 

void usart_init(void){
    UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALER);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
    }

/*void usart_send( unsigned char data){
    while(!(UCSR0A & (1<<UDRE0))); //wait for transmit buffer
    UDR0 = data; //data to be sent
}*/

unsigned char usart_receive(void){
    while(!(UCSR0A & (1<<RXC0))){PORTB|=(0<<5);}; //wait for new data
    PORTB|=(1<<5);
    return UDR0; //received data
}

