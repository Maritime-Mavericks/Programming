// right one
#include <stdio.h>
#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU / 16 / BAUD - 1)

void USART_Transmit(unsigned char data);

volatile char received_data;
volatile int digitNumber = 0;
volatile int latOrLong = 0; // 0 is undefined, 17 is latitude, 31 is longitude
volatile int latDigits[8];
volatile int longDigits[7];
volatile int coordinateReceived = 0;

int main(void){
    
    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

    // Setup USART for RF
    UBRR2H = (unsigned char)(MYUBRR >> 8);
    UBRR2L = (unsigned char) MYUBRR;
    UCSR2B = (1 << RXEN2) | (1 << TXEN2);
    UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);

    UCSR2B |= (1<<RXCIE2); //enable interrupts for RXIE
    sei(); //enable interrupts

    while(1){ // empty infinite loop
        if(coordinateReceived == 1){
            double receivedLatitude = 0;
            double receivedLongitude = 0;

            // Compute latitude
            receivedLatitude += (
                latDigits[0] * 10 +
                latDigits[1] * 1 +
                latDigits[2] * 0.1 +
                latDigits[3] * 0.01 +
                latDigits[4] * 0.001 +
                latDigits[5] * 0.0001 +
                latDigits[6] * 0.00001 + 
                latDigits[7] * 0.000001
            );

            // Compute longitude
            receivedLongitude += (
                longDigits[0] * 1 +
                longDigits[1] * 0.1 +
                longDigits[2] * 0.01 +
                longDigits[3] * 0.001 +
                longDigits[4] * 0.0001 +
                longDigits[5] * 0.00001 +
                longDigits[6] * 0.000001
            );

            printf("Lat: %f    Long: %f\n", receivedLatitude, receivedLongitude);

            coordinateReceived = 0;
        }
    } 

    return 0;
}

ISR(USART2_RX_vect){
    received_data = UDR2;
    int digit = received_data - 48;
    // float x = pow(10, -6) * digit;
    // printf("float point: %f\n", x);

    if(digit == 17){ // latitude
        coordinateReceived = 0; // reset coordinate received
        latOrLong = 17;
        digitNumber = 0;
    } else if(digit == 31){ // longitude 
        latOrLong = 31;
        digitNumber = 0;
    } else if(digit == 22){ // finished
        coordinateReceived = 1;
        latOrLong = 0;
    } else { // if you received any other digit which would be the coordinate digits
        if(latOrLong == 17){ // latitude
            latDigits[digitNumber] = digit;
            digitNumber++;
        } else if(latOrLong == 31){ // longitude
            longDigits[digitNumber] = digit;
            digitNumber++;
        }
    }
}
