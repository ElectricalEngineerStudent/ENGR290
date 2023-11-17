#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>


#ifdef __cplusplus //because c++ compile
 extern "C" {      // 
#endif             //until here



void pwm_init_PD3();
void pwm_PD3_write(uint8_t);



#ifdef __cplusplus    //end of c++ directive for C
}                     //
#endif                //until here