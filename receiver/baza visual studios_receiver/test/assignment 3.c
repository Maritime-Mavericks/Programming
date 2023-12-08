#define F_CPU 16000000UL

#include <stdio.h>// + delay, + usart, + io
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
unsigned char x, y, z1, z2; // inputs and outputs
unsigned char q0, q1, q2, q0_next, q1_next, q2_next;
int state=0;
void read_xy_values(void); //checking which button is pressed
void show_output(void); //showing current state + state variables on the screen
void state_transition(void); //advancing to the new state by implementing transition equations

// int button1 = 0b00111110;
// int button2 = 0b00111101;
// int button3 = 0b00111011;
// int button4 = 0b00110111;

// int LED1 = 0b00010000;
// int LED2 = 0b00100000;
// int LED3 = 0b01000000;
// int LED4 = 0b10000000;


int main() {    

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
    // 2 step process to enable the LCD
    // initialize the 12c communication 
    i2c_init(); 
    // initialize the LCD 
    //LCD_init(); 
    // initialize the temperature sensor
    lm75_init(); 
    // configuration of the LEDs
    DDRD = 0xFF; // I/O board: PD4...7 as output, for LEDs
    PORTD = 0x00; // Set output LEDs to off

    // configuration of the input buttons
    DDRC = 0xF0; //I/O board PC0…3 as inputs, for buttons
    PORTC = 0x3F; // Enable internal pull at PC0..3 inputs
    
    q0 = 0;
    q1 = 0;
    q2 = 0;
   
   while(1)
    {
    read_xy_values();
    state_transition();
    show_output();
    _delay_ms(1000);
    }

}
void read_xy_values(void){
    if(PINC == 0b00111011){//button 3
        x=1;
    }
    else{
        x=0;
    }
    if(PINC == 0b00110111){//button 4
        y=1;
    }
    else{
        y=0;
    }
    if(PINC == 0b00110011){//button 3 and 4
        x=1;
        y=1;
    }
}
void state_transition(void){//calculations
    q0_next=((!q1)&&x)||(q0&&(!x))||q2;
    q1_next=(((!q2)&&q0&&x))||(q1&&(!x))||(q1&&q2);
    q2_next=(q2&&(!q0))||((!q0)&&(!x)&&y);
    z1=((!q0_next)||(!q1_next)||q2_next);
    z2=(q1&&q2)||(q2&&(!q0));
    q0=q0_next;//changing values of q0,q1,q2
    q1=q1_next;
    q2=q2_next;
    state=(q2<<2|q1<<1|q0);//making q0,q1,q2 as one bit number
}
void show_output(void){
    printf("X: %d, Y: %d, Z1: %d, Z2: %d, Q0: %d, Q1: %d, Q2: %d,", x, y, z1,z2,q0,q1,q2);//printing functions
    // printf("state: %d\n", state);
    printf("%c\n", 'A'+state);//the state 
}