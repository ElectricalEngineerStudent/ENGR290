#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#ifdef __cplusplus //because c++ compile
extern "C" {      //
	#endif             //until here
	void uart_tx_init();
	void uart_send_byte(uint8_t);
	void uart_print_uint8(uint8_t);
	#ifdef __cplusplus    //end of c++ directive for C
}                     //
#endif                //until here