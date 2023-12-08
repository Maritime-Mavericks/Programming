#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#include "lcd.h"
#include <avr/eeprom.h>
#define BAUDRATE 19200
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 8UL)))-1)


unsigned int address = 8;
unsigned int address1 = 1;

//function prototypes
void init_timer1(void);
void ninit_adc(void);
uint16_t read_adc(uint8_t channel);

void usart_init(void);
unsigned char usart_receive(void);
void usart_send(unsigned char data);

int check_limb(int limb_to_check);

volatile uint16_t timer_ticks=0; //volatile for working with interrupts

volatile int limb_to_try;
int successfull_movements [5];
int no_of_elemtsw=0;

//int result_adc;

int main(void){
    //configuer leds
   // DDRD = 0xFF; // PD0...PD7 are all outputs, PD4..PD7 are leds
   // PORTD = 0xF0; //inicially leds on

   DDRC = 0b00110000; //PC0..PC4 are inputs
   PORTC = 0b00110000; //the pull ups are unabled
   DDRB = (0<<5); // set bit 5 at ddrb low as output, so it is low from the begginning(PB5)

    srand(64);
    i2c_init(); //initialize 12c communication
    LCD_init(); // initialize LCD_init

    init_timer1();
    init_adc();
    int k=0;
    int sequence =0;
    sequence = eeprom_read_byte((uint8_t *)address); // read a byte stored at the address 0 in the eeprom memory'
while(1){
    
    if(timer_ticks==2000){
    for(k; k<5;k++){
    LCD_set_cursor(k,0);
    printf("%d", limb_to_try);
    
    //LCD_set_cursor(0,2);
    //printf("successfull %d",check_limb(limb_to_try));

    if (check_limb(limb_to_try)){//add this element to the array
        successfull_movements[no_of_elemtsw]=limb_to_try;

    if(no_of_elemtsw==5){
        no_of_elemtsw=0; //when we have 5 successfull tries we reset the array
    }
    no_of_elemtsw++;
       
    }
    else{//unseccessfull movement, delete all previous movements
        no_of_elemtsw =0;
        k=0;

    }
    
    

    for(int i=0; i<no_of_elemtsw; i++){
        LCD_set_cursor(k,1);
        printf("%d", i); //which part of the array we are checking
        LCD_set_cursor(k,3);
        printf("%d", successfull_movements[i]); //how many successfull movements was in the array
        if(check_limb(limb_to_try)==1){
        eeprom_write_byte((uint8_t *)address, (uint8_t)successfull_movements[i]);
        PORTB |= (1<<PB5);
    }
        else{PORTB &= ~(1<<PB5);}
        usart_send(successfull_movements[i]);
    }
        
    }
    k=0;
    }
}


}

void init_timer1(){
    //set up CTC mode
    TCCR1B |= (1<<WGM12)|(1<<CS11)|(1<<CS10); // WGM12=ctc mode, cs11+cs10=prescale mode
    //set up the number of ticks
    OCR1A = 249; //I need to count to 250 ticks
    TIMSK1 |= (1<<OCIE1A); //enable interrupts for matching ORC1A
    sei(); // enable global interrupts
}

ISR(TIMER1_COMPA_vect){//this will fire every 1 ms; do not print in interrupts!!!
    if(timer_ticks == 2000){
        PORTD ^= 0b11110000; // toggle PD4..PD7, leds are there
        limb_to_try=(rand()%4)+1;// the rand()%4 would generate 0,1,2,3 we need 1,2,3,4 so that is why plus 1
        timer_ticks =0; // reset and start over with counting the ticks
    } 
    else{
        //increment the ms counter
        timer_ticks++;
    }
}

void init_adc(){
    // Select Vref= AVcc
    ADMUX = (1<<REFS0);
    //set prescalerto 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

uint16_t read_adc(uint8_t channel){
    ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
    ADMUX |= channel; // set the desired channel
    //start a conversion
    ADCSRA |= (1<<ADSC);
    // now wait for the conversion to complete
    while ( (ADCSRA & (1<<ADSC)) );
    // now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return ADC;
}

int check_limb(int limb_to_check){
    int result_adc = read_adc(limb_to_check-1)*4.96;//the adc channels are 0,1,2,3 (pins we read); result in mV
    LCD_set_cursor(0,2);
    printf("%d", result_adc); //show the voltage on the current limb
    if (result_adc>1000){
        return 1;
    }
    else{
        return 0;
    }
    if(UDR0==1){
        usart_send(check_limb(limb_to_try));
    }
    }
    
void usart_init(void){
    UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALER);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
    }

void usart_send( unsigned char data){
    while(!(UCSR0A & (1<<UDRE0))); //wait for transmit buffer
    UDR0 = data; //data to be sent
}

unsigned char usart_receive(void){
    while(!(UCSR0A & (1<<RXC0))); //wait for new data
    if(UDR0 == 2){
        eeprom_write_byte((uint8_t *)address1, (uint8_t)UDR0);
    }
    return UDR0; //received data
}
 