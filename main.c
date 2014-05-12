/* -----------------------------------------------------------------------
 * Title: Led blinking reaction on pressed switch (I/O)
 * Hardware: ATtin85
 -----------------------------------------------------------------------*/



#define F_CPU 1000000UL // Define software reference clock for delay duration
// Must be write before call delay.h
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <string.h>
#include "manchester.h"
uint8_t moo = 1; //last led status
uint8_t data[13];
Manchester man;
/*int main( void )
{
  init(&man, 100);
  DDRB = (1<<PB4) | (1 << PB3);
  workAround1MhzTinyCore(&man, 1);
  setupTransmit(&man, (1<<PB3),MAN_1200);
  while(1){
    send(&man, 100, "Hello World!", 12);
    PORTB ^= (1<<PB4);
    _delay_ms(10);
  }
}*/


int main( void) {
  sei();
  init(&man, 100);
  DDRB = (1<<PB4) | (0<< PB3);
  setupReceive(&man, (1<<PB3), MAN_1200);
while(1){

 if (recv(&man, &data, 12) ){
   if(strncmp("Hello World!", data, 12) == 0) PORTB ^= (1 << PB4);
  }
}
}
