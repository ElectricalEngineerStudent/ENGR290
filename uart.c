#define F_CPU 16000000UL
#define BAUD 9600UL
#define UBRR_reg ((F_CPU)/((BAUD)*(16UL))-1)
#include "uart.h"
volatile static uint8_t uart_tx_busy = 1;

ISR(USART_TX_vect){
  uart_tx_busy = 1;
}

void uart_tx_init() {
  UBRR0H = (uint8_t)((UBRR_reg)>>8); // Set the UART speed as defined by UBRR_reg
  UBRR0L = (uint8_t)UBRR_reg;
  UCSR0B|=(1<<TXCIE0)|(1<<TXEN0); // Enable TX and TX interrupt
  UCSR0C=(3<<UCSZ00); // 8N1
}

void uart_send_byte(uint8_t c){
  while(uart_tx_busy == 0);
  uart_tx_busy = 0;
  UDR0 = c;
}

void uart_print_uint16(uint16_t num) 
 { 
  uint8_t a,b,c,d;
  if(num >9999)
  {
    uart_send_byte(9+48);
    uart_send_byte(9+48);
    uart_send_byte(9+48);
    uart_send_byte(9+48);
  }
  else if(num>=1000)
  {
    a=num/1000;
    b=num/100-10*a;
    c=num/10-10*num/100;
    d=num%10;
    uart_send_byte(a+48);
    uart_send_byte(b+48);
    uart_send_byte(c+48);
    uart_send_byte(d+48);
  }
  else if(num>=100)
  {
    a=num/100;
    b=num/10-10*a;
    c=num%10;
    uart_send_byte(a+48);
    uart_send_byte(b+48);
    uart_send_byte(c+48);
  }
  else if(num>=10)
  {
     b=num/10;
     c=num%10;
     uart_send_byte(b+48);
     uart_send_byte(c+48);
  }
  else
  {
    c=num;
    uart_send_byte(c+48);

  }

 }