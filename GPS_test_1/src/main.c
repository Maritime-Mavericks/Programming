#define F_CPU 16000000UL

#include <stdio.h>
#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#include "minmea.h"

#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void USART_init(void);
volatile char received_data;
char received_string[512]; // Maximum string length, adjust as needed
volatile uint8_t string_index = 0;

int main(void){

    DDRD  = 0x0F; // I/O board:PD4...7 as outputs, for LEDs
    DDRC  = 0xF0; // I/O board PC0...3 as inputs, for buttons
    PORTC = 0x3F; // Enable internal pull at PC0..3 inputs
    PORTD = 0x00; // Set output LEDs to off
    
    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication

    // i2c_init();
    // lm75_init();
    // LCD_init();
    // get_temperature();
    // LCD_set_cursor(0, 0);

    USART_init(); //Call the USART initialization code
    UCSR3B |= (1<<RXCIE3); //enable interrupts for RXIE
    sei(); //enable interrupts

    while(1){ // empty infinite loop
    } 

    return 0;
}

ISR(USART3_RX_vect){
    received_data = UDR3;
    if (received_data == '\n') {
        // Parse formed line
        if(strstr(received_string, "GPR")){
            printf("%s\n", received_string);
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, received_string)) {
                printf(
                    "Lat-Lon-Speed: %f -- %f -- %f\n\n",
                    minmea_tocoord(&frame.latitude),
                    minmea_tocoord(&frame.longitude),
                    minmea_tofloat(&frame.speed)
                );
            }
            else {
                printf("Error! Not parsed correctly!");
            }
        }

        // Store a new line
        received_string[string_index] = '\0'; // Null-terminate the string
        string_index = 0; // Reset the string index
    } else {
        received_string[string_index++] = received_data;
    }
}

void USART_init(void){
    UBRR3H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR3L = (uint8_t)(BAUD_PRESCALER);
    UCSR3B = (1<<RXEN3)|(1<<TXEN3);
    UCSR3C = ((1<<UCSZ30)|(1<<UCSZ31));
}

