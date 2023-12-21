// right one
#include <stdio.h>
#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>

#include "minmea.h"

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU / 16 / BAUD - 1)

void USART_Transmit(unsigned char data);

int main(void){
    
    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

    // Setup button
    DDRD &= ~(1 << DD5); // Set DD5 as input
    //PORTD |= (1 << DD5); // Pull up enabled on DD5

    // Setup USART for RF
    UBRR0H = (unsigned char)(MYUBRR >> 8);
    UBRR0L = (unsigned char) MYUBRR;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    UCSR0B |= (1<<RXCIE0); //enable interrupts for RXIE
    sei(); //enable interrupts
    
    while(1){ // empty infinite loop
        if((PIND & (1 << DD5)) == (1 << DD5)){
            USART_Transmit(80); // 80 is "P": STOP/PAUSE
            _delay_ms(1000);
        }
    } 

    return 0;
}

void USART_Transmit(unsigned char data){
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

