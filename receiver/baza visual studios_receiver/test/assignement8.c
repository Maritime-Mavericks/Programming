#define F_CPU 16000000UL
#include <stdio.h>
#include <usart.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

#include "i2cmaster.h"
#include "lcd.h"
#include <stdint.h>
#define ADC_PIN0 0 //what is the ADC channel we’ll use, here A0
#define ADC_PIN1 1 //what is the ADC channel we’ll use, here A1
#define ADC_PIN2 2 //what is the ADC channel we’ll use, here A2
#define ADC_PIN3 3 //what is the ADC channel we’ll use, here A3
//function prototypes
uint16_t adc_read(uint8_t adc_channel);
int BTN1 = 0x3E;
int BTN2 = 0x3D;
int BTN3 = 0x3B;
int BTN4 = 0x37;

float Vin;
  
int show(float, int, int);


int main(void){
    uart_init(); // open the communication to the microcontroller
    //usart_init();        //Call the USART initialization code
    io_redirect(); // redirect input and output to the communication
    // i2c_init(); //lcd
    // LCD_init();//lcd

    DDRB = (1<<5); // set 5 bit at ddrb high as output, so it is high from the begginning
    



    uint16_t adc_result0;
    uint16_t adc_result1;
    uint16_t adc_result2;
    uint16_t adc_result3;
    int a=0;
    int b=1;
    int c=2;
    int d=3;
    // Select Vref= AVcc
    ADMUX = (1<<REFS0); 
    //set prescalerto 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); 

    while(1){
       

        show(adc_result0, ADC_PIN0, a);
       // show(adc_result1, ADC_PIN1, b);
        //show(adc_result2, ADC_PIN2, c);
        //show(adc_result3, ADC_PIN3, d);
        _delay_ms(2000);
        //LCD_clear;
        }

return 0;
 
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
    ADMUX |= adc_channel; // set the desired channel 
    //start a conversion
    ADCSRA |= (1<<ADSC);
    // now wait for the conversion to complete
    while ( (ADCSRA & (1<<ADSC)) );
    // now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return ADC;
    }

int show(float adc_result, int ADC_PIN, int number){
        adc_result = adc_read(ADC_PIN);
        Vin = ((float)adc_result/1024.0)*5;
        if(Vin>1){
            PORTB |= (1<<PB5); // if it was on it will be on, if it was off it will be on //turn on the PB5 on the arduino
        }
        else{
            PORTB &= ~(1<<PB5);//turn off the PB5 on te arduino
        }

        
        //LCD_set_cursor(0, number);
        printf("%.4f V\n", Vin);
        _delay_ms(1000);
}

