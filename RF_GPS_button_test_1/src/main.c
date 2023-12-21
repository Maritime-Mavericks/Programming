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

volatile char received_data;
char received_string[512]; // Maximum string length, adjust as needed
volatile uint8_t string_index = 0;

volatile double latitude;
volatile double longitude;
volatile int receivedNewCoord = 0;

volatile int startRescueTransmission = 0;

void USART_Transmit(unsigned char data);

int main(void){
    
    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

    // Setup button
    DDRD &= ~(1 << DD5); // Set DD5 as input
    PORTD |= (1 << DD5); // Pull up enabled on DD5

    // Setup USART for RF and GPS
    UBRR0H = (unsigned char)(MYUBRR >> 8);
    UBRR0L = (unsigned char) MYUBRR;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    UCSR0B |= (1<<RXCIE0); //enable interrupts for RXIE
    sei(); //enable interrupts

    while(1){ // empty infinite loop
        if((PIND & (1 << DD5)) == (1 << DD5) && startRescueTransmission == 0){
            USART_Transmit(83); // 83 is "S": Start
            startRescueTransmission = 1;
        }

        if(startRescueTransmission == 1){
            if(receivedNewCoord == 1){
                // Hard-code for testing
                latitude = 54.912581;
                longitude = 9.780309;

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

                // Send latitude 
                USART_Transmit(65); // 65 is 'A': Latitude
                _delay_ms(10);
                for(int i = 0; i < 8; i++){
                    char digit = latDigits[i] + 48;
                    USART_Transmit(digit);
                    _delay_ms(10);
                }
                // Send longitude
                USART_Transmit(79); // 65 is 'O': Longitude
                _delay_ms(10);
                for(int i = 0; i < 7; i++){
                    char digit = longDigits[i] + 48;
                    USART_Transmit(digit);
                    _delay_ms(10);
                }
                // Send finished
                USART_Transmit(70); // 70 is 'F': Finish

                receivedNewCoord = 0;
            }
        }
    } 

    return 0;
}

ISR(USART_RX_vect){
    received_data = UDR0;
    if (received_data == '\n') {
        // Parse formed line
        if(strstr(received_string, "GPR")){
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, received_string)) {
                latitude = minmea_tocoord(&frame.latitude);
				longitude = minmea_tocoord(&frame.longitude);
                receivedNewCoord = 1;
            } // else there is an error
        }

        // Store a new line
        received_string[string_index] = '\0'; // Null-terminate the string
        string_index = 0; // Reset the string index
    } else {
        received_string[string_index++] = received_data;
    }
}

void USART_Transmit(unsigned char data){
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

