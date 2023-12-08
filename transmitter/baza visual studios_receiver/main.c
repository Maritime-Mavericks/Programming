
#include <stdio.h>

#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

void USART_Transmit(unsigned char data)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

int main(void)
{
    USART_Init(MYUBRR);

    while(1)
    {
        // USART_Transmit('H');
        // USART_Transmit('e');
        // USART_Transmit('l');
        // USART_Transmit('l');
        // USART_Transmit('o');
        // USART_Transmit(' ');
        // USART_Transmit('W');
        // USART_Transmit('o');
        // USART_Transmit('r');
        // USART_Transmit('l');
        // USART_Transmit('d');
        // USART_Transmit('!');
        // USART_Transmit('\n');
        USART_Transmit('2');
        _delay_ms(200);
        USART_Transmit('3');
        _delay_ms(200);
        USART_Transmit('4');
        USART_Transmit('\n');
        _delay_ms(1000);
    }
}

