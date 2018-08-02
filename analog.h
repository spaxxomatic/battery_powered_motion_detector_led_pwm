
#ifndef _ADC_H_
	#define _ADC_H_	

	#include <avr/io.h>
	#include <avr/interrupt.h>
	
	#define ANALOG_OFF ADCSRA=0
	#define ANALOG_ON  ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);	

#endif //_ADC_H_






