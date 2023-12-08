#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#define dirPin (1<<PD9)
#define stepPin (1<<PD8)

void rotate_cw(void);
void rotate_acw(void);

int main()
 { 
   // setup dirPin and stepPin as output
    DDRD |= dirPin | stepPin;

   while (1){
        
        rotate_cw();
        _delay_ms(1000);
        rotate_acw();
        _delay_ms(1000);
        
    }
   return 0;
 }

void rotate_cw(void){
    unsigned char i;
    //send High pulse for clockwise direction
    PORTD |= dirPin;
    //send 200 pulses to rotate One full cycle
    for(i=0; i<200; i++){
        PORTD |= stepPin;    
        _delay_ms(50);
        PORTD &= ~stepPin;
        _delay_ms(50);
    }
}

void rotate_acw(void){
unsigned char i;
    //send low pulse for anti-clockwise direction
PORTD &= ~dirPin;
    //send 200 pulses to rotate One full cycle
    for(i=0; i<200; i++){
        PORTD |= stepPin;    
        _delay_ms(50);
        PORTD &= ~stepPin;
        _delay_ms(50);
    }
} 