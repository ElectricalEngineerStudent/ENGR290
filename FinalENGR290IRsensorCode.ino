

//Code inspired by Github.
//Code modified by Team 3, ENGR290.
//IR use Connector P5

#include <avr/io.h>

unsigned int duty;
 double distance;
unsigned int ADC_read()

{

ADMUX = 0x40;        //channel A0 selected
ADCSRA|=(1<<ADSC);   // start conversion
while(!(ADCSRA & (1<<ADIF)));   
ADCSRA|=(1<<ADIF);  
return (ADC); //return calculated ADC value

} 



int main(void)

{
Serial.begin(9600);

ADMUX=(1<<REFS0);      // Selecting internal reference voltage

ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     // Enable ADC also set Prescaler as 128

int i = 0; // define an integer to save adc read value
DDRD |= (1 << DDD3);


 TCCR2A |= (1<<WGM20)|(1<<WGM21)|(0<<COM2B0)|(1<<COM2B1);              


 TCCR2B |=(1<<CS20)|(0<<CS21)|(0<<CS22);  


    while (1) 
    {
      i = ADC_read();  //save adc read value in integer
      
      distance=1/(((i*5.0)/1024-0.24)/(20.4));
      
      if(Serial.available()!=0)
      {
        Serial.print(i);
      Serial.print("     distance (cm):");
      Serial.println(distance);
      }

      
      if(distance<15)
      {
        duty=0;
      }
      else if(distance>40)
      {
        duty=255;
      }
      else duty=distance*10.2-153;

      Serial.print("Distance: ");
      Serial.println(duty);
      OCR2B=duty;
      _delay_ms(200);
    
    }      
}