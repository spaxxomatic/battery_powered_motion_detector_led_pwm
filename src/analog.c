
#include "analog.h"

#if USE_ADC
volatile unsigned char channel = 0;


//------------------------------------------------------------------------------
//
void ADC_Init(void)
{ 
	ADMUX = (1<<REFS0);	
	//Free Running Mode, Division Factor 128, Interrupt on
	ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);		

}

//------------------------------------------------------------------------------
//
ISR (ADC_vect)
{
    ANALOG_OFF; //ADC OFF
	//adc_array[channel++][idx % DAC_AVERAGE] = ADC;
	adc_array[channel] = ADC;	
	channel++;	
	if (channel >= LASTANALOGINPUT) {channel = 0;}
    ADMUX = (1<<REFS0) + channel;
    if (channel) //keep converting until all channels read
	{
		ANALOG_ON;
	}		
}

#endif //USE_ADC


