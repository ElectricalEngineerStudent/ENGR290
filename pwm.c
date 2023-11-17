#include "pwm.h"

void pwm_init_PD3(){
  DDRD |= (1 << DDD3);

  TCCR2A |= (1<<WGM20)|(1<<WGM21)|(0<<COM2B0)|(1<<COM2B1);
  TCCR2B |=(1<<CS20)|(0<<CS21)|(0<<CS22);
  
  }
void pwm_PD3_write(uint8_t val){
  OCR2B = val;        //set value into timer register
  }