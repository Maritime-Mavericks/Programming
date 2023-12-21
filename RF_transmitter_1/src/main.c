// left one
#include <stdio.h>
#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU / 16 / BAUD - 1)

void USART_Transmit(unsigned char data);
volatile char received_data;

int main(void){
    
    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

    // Setup USART for RF
    UBRR0H = (unsigned char)(MYUBRR >> 8);
    UBRR0L = (unsigned char) MYUBRR;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    UCSR0B |= (1<<RXCIE0); //enable interrupts for RXIE
    sei(); //enable interrupts

    double latitude = 12.345673; // 8 digits
    double longitude = 2.468123; // 7 digits
    int latDigits[8];
    int longDigits[7];

    // Get latitude digits
    latDigits[0] = ((long int)(latitude) % 100) / 10;
    latDigits[1] = ((long int)(latitude) % 10) / 1;
    latDigits[2] = ((long int)(latitude * 10)) % 10;
    latDigits[3] = ((long int)(latitude * 100)) % 10;
    latDigits[4] = ((long int)(latitude * 1000)) % 10;
    latDigits[5] = ((long int)(latitude * 10000)) % 10;
    latDigits[6] = ((long int)(latitude * 100000)) % 10;
    latDigits[7] = ((long int)(latitude * 1000000)) % 10;

    // Get longitude digits
    longDigits[0] = ((long int)(longitude) % 10) / 1;
    longDigits[1] = ((long int)(longitude * 10)) % 10;
    longDigits[2] = ((long int)(longitude * 100)) % 10;
    longDigits[3] = ((long int)(longitude * 1000)) % 10;
    longDigits[4] = ((long int)(longitude * 10000)) % 10;
    longDigits[5] = ((long int)(longitude * 100000)) % 10;
    longDigits[6] = ((long int)(longitude * 1000000)) % 10;

    while(1){ // empty infinite loop
        // Send latitude 
        USART_Transmit(65); // 65 is 'A'
        for(int i = 0; i < 8; i++){
            char digit = latDigits[i] + 48;
            USART_Transmit(digit);
            _delay_ms(100);
        }
        // Send longitude
        USART_Transmit(79); // 65 is 'O'
        for(int i = 0; i < 7; i++){
            char digit = longDigits[i] + 48;
            USART_Transmit(digit);
            _delay_ms(100);
        }
        // Send finished
        USART_Transmit(70); // 70 is 'F'

        _delay_ms(2000);
    } 

    return 0;
}

ISR(USART_RX_vect){
    received_data = UDR0;
}

void USART_Transmit(unsigned char data){
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

