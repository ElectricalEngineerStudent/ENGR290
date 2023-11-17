


// Team 3 modified code for US sensor. Code inspired by github
 //PWM LED PD3/D3:   US echo PD2/D2 and trigger PB3/D11
 
#define F_CPU 16000000UL

//#include <xc.h>
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"
#include "pwm.h"
#include "uart.h"

volatile unsigned short impulse;
volatile uint16_t travel_time;

// Port Initialization
void Port_Init()
{

	
	DDRB|=(1<<DDB3)|(1<<DDB5); //set trigger pin
	
	EIMSK |= (1<<INT0); //enable external interrupts
	EICRA |= (1<<ISC00); // Any logical change on INT0 generates an interrupt

	//Config and enable Timer1
	TCCR1B = (0<<CS12)|(1<<CS11)|(1<<CS10);
}

ISR( INT0_vect )
{
	
	if(PIND & (1 << 2)) {TCNT1 = 0;}/* Clear Timer counter */
	else  {impulse = TCNT1;}
	

}

unsigned char us_read( void )
{
	
	impulse = 0;
	
	// send trigger pulse of 10 us to the HC-SR04 Trig Pin.
	PORTB |=  (1<<DDB3);
	_delay_us( 10 );
	PORTB &= ~(1<<DDB3);


	while( impulse == 0 );
	
	if (impulse<3700) {travel_time=(4 * impulse/2); return (0.000004 * impulse/2 * 34300);} else {travel_time=(0);return 255;} // calculate time and then multiply by 34300 cm/s
	
}


int main(void){
	uint8_t dist, duty,a,b,c;
	pwm_init_PD3();
	Port_Init();
	uart_tx_init();
	sei();
	
	while(true){
		dist = us_read();
		_delay_ms(10);
/*
		if(dist<15){duty=0; PORTB|=(1<<DDB5);}
		else if(dist>40){duty=255; PORTB|=(1<<DDB5);}
		else {duty=((dist-15)*10); PORTB&=~(1<<DDB5);}
*/
    if(dist>=40){
      duty=0; PORTB|=(1<<DDB5);
      }
		else if(dist<=15){
      duty=255; PORTB|=(1<<DDB5);
      }
		else if(dist<40&dist>15)
    {
      duty=(-10*(dist-40)); PORTB&=~(1<<DDB5);
    }
		pwm_PD3_write(duty);

      uart_send_byte('I');
			uart_print_uint16(travel_time);
			uart_send_byte(32);
			uart_send_byte('D');
			uart_print_uint16(dist);
			uart_send_byte(10);


	

		
	}

	
}