#include <stdio.h>

#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

unsigned char USART_Receive(void)
{
    while(!(UCSR0A & (1<<RXC0)));
    return UDR0;
}
void USART_Transmit(unsigned int data)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

int main(void)
{
   
    USART_Init(MYUBRR);

    DDRB = (1<<5);
    
    while(1)
    {
        
        unsigned char data = USART_Receive();
           
      
        USART_Transmit(data);
       //PORTB|=(1<<5);
        // if(data==50){
        //     PORTB|=(1<<5);
        // }
        //  if(data==49){
        //     PORTB=(0<<5);
        // }
     
    }
}
