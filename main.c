/*************************************************************************
Title:    Motion sensor controlled light. On motion, an LED light is dimmed up. If no motion for a certain time, LED will be dimmed down
Author:   Lucian Nutiu <lucian.nutiu@gmail.com>  
File:     $Id: mainc,v 1.6 2004/12/10 13:53:59 peter Exp $
Software: AVR-GCC 3.3
Hardware schematic: see motion_controlled_sensor_with_pwm.sch
**************************************************************************/
#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "analog.h"
 

#define BAUDRATE 9600
#include "usart_send.h"

volatile unsigned char channel = 0;

volatile uint16_t adc_value = 0;

#define MOTION_SENSOR_INPUT PINB4

void adc_init(void)
{
//select reference voltage
//AVCC with external capacitor at AREF pin
//ADMUX|=(0<<REFS1)|(1<<REFS0);

//internal 2,56 V reference
ADMUX|=(1<<REFS1)|(1<<REFS0);

//set prescaller and enable ADC
ADCSRA|=(1<<ADEN)|(1<<ADIE);//enable ADC with dummy conversion
//set sleep mode for ADC noise reduction conversion
//set_sleep_mode(SLEEP_MODE_ADC);
}

volatile uint16_t counter_tick = 0;
char ticker;


#define TIMER_0_FREQ 30
#define KEEP_LIGHT_ON_DURATION TIMER_0_FREQ*3

#define POWER_SUPPLY_ENABLE_PIN PB0
#define BAT_V_CHECK_PIN PC1
#define LIGHT_ON_TIMER_SEC 100

ISR(TIMER0_OVF_vect) //clk/1024/256 -> ~30 Hz
{
	cli();
	if (counter_tick%TIMER_0_FREQ >TIMER_0_FREQ/2) {ticker = '*';}
	else {ticker = 0;};
	counter_tick += 1;
	sei();
	//PORTB ^= (1 << BAT_V_CHECK_PIN); 
}

volatile uint8_t adc_irq_flag = 0;
ISR (ADC_vect)
{
	cli();
	//adc_value = ADCL; 
	//adc_value += (ADCH<<8); 
	adc_value = ADC; 
	adc_irq_flag += 1;
	sei();
}

void adc_start_conversion(uint8_t channel)
{
ADMUX=(ADMUX&0xF0)|channel;
//Start conversion with Interrupt after conversion
//enable global interrupts
//sei();
//ADCSRA |= (1<<ADSC)|(1<<ADIE)|(1<<ADFR)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
ADCSRA |= (1<<ADSC)|(1<<ADIE)|(0<<ADFR)|(1<<ADPS1)|(1<<ADPS0);
}


/*void wait_until_key_pressed(void)
{
    unsigned char temp1, temp2;
    unsigned int i;
    
    do {
        temp1 = PINB;                  // read input
        for(i=0;i<65535;i++);
        temp2 = PINB;                  // read input
        temp1 = (temp1 & temp2);       // debounce input
    } while ( temp1 & _BV(PINB4) );
    
    loop_until_bit_is_set(PINB,PINB4);            //
}
*/

/*
void timer_init(void){

   //Timer1 is used as 1 sec time base
   //Timer Clock = 1/1024 of sys clock
   //Mode = CTC (Clear Timer On Compare)
   TCCR1B|=((1<<WGM12)|(1<<CS12)|(1<<CS10));
   //Compare value=976
   OCR1A=976;
   TIMSK|=(1<<OCIE1A);  //Output compare 1A interrupt enable
 
}
*/
//timer 0 is used to control the on time after a motion event
void start_timer0(void){
TCCR0 |= (1<<CS02)|(1<<CS00); //prescaler 1024
TIMSK |= (1<<TOIE0); //overflow interrupt
}

void stop_timer0(void){
TCCR0 = 0x00; //stops the timer
TIMSK &= ~(1<<TOIE0); //disable interrupt
}

volatile uint8_t int0_irq_flag = 0;
ISR(INT0_vect)
{
   PORTB ^= (1 << PB1);    // debugging
   int0_irq_flag += 1;
}

char buff[20] = "";


void enable_pwm(void){
	TCCR1A = (1<<WGM10) | (1<<COM1A1); // PWM, phase correct, 8 bit.
	//TCCR1B = (1<<CS12) | (1<<CS10); // Prescaler 1024 
	TCCR1B = (1<<CS11) | (1<<CS10); // Prescaler 64 
	//TCCR1B =  (1<<CS11); // Prescaler 8 
}

void setup(void){
 
    MCUCR &= ~((1 << ISC01) | (1 << ISC00));// level interrupt INT0 (low level)
	//MCUCR |= ((1 << ISC01) | (1 << ISC00)); //rising edge does not work as an wake-up interrupt when in sleep mode
	
	//PORTB |= (1 << PB1); // PULLUP für inputs
	//DDRB &=~ (1 << MOTION_SENSOR_INPUT) ;    // Activate input
	DDRB |= (1 << PB1)|(1 << POWER_SUPPLY_ENABLE_PIN);    // Activate output ,PB0 is power supply relais, PB1 is OC1A (PWM output)
	DDRC |= (1 << BAT_V_CHECK_PIN);    //The pin will be put high when measuring the battery voltage
	PORTD |= (1 << PD2); // PULLUP für int0 pin
	//DDRB |= (1<< OC1A); // Port OC1A mit angeschlossener LED als Ausgang
	enable_pwm();
	adc_init();
	usart_init(BAUDRATE); // setup the UART
	//timer_init();	
	sei();
	usart_write_str("INIT OK");
	

}

uint8_t pwmval = 0;

uint8_t output_state = 0;
#define DIM_STEP_DUR 20
#define FULL_ON 1
#define FULL_OFF 0

void dim_up(void){
	while(pwmval < 0xFF){
		OCR1A = ++pwmval; 
		if (pwmval < 50){
		_delay_ms(DIM_STEP_DUR*2);
		}else if (pwmval < 100){
		_delay_ms(DIM_STEP_DUR*2);
		}else{
		_delay_ms(DIM_STEP_DUR*2);
		}
		wdt_reset();
	}
	output_state = FULL_ON;
}

bool dim_down(void){
	while(pwmval >0){
		if ( !(PIND & (1<<PD2)) ){ //PIND2 is int0  - if low, stop dimming
			return false;
		};
		OCR1A = --pwmval;
		_delay_ms(DIM_STEP_DUR);
		wdt_reset();
	}
	output_state = FULL_OFF;
	return true;
}

//internal ref = 2,56 V, bat has a 1/2 divider, discharged when at 3.2V
//1,6V after divider => 1,6/2,56*1024 = 240
//loaded at 4,1V => 2,05 after divider => 2,05/2,56*1024 = 240
#define LEVEL_BAT_DISCHARGED  690
#define LEVEL_BAT_LOADED 920
#define ENABLE_INT0_ON_LEVEL_CHANGE 

#define BAT_DISCHARGED 0
#define BAT_OK 1
#define BAT_FULL 2

#define STATE_TIMER_COUNTDOWN 1
#define STATE_SLEEPING 2
#define STATE_MOTION_DETECTED 4


void enable_int0_level(void){
	MCUCR &= ~((1 << ISC01) | (1 << ISC00));// level interrupt INT0 (low level)
	GICR |= (1 << INT0); 
}

void enable_int0_downendge(void){
	MCUCR &= ~((1 << ISC01) | (1 << ISC00));// level interrupt INT0 (low level)
	GICR |= (1 << INT0); 
}

void disable_int0(void){
	GICR &= ~(1 << INT0); //disable int0 external interrrupt
}

uint8_t get_battery_state(void){
		
		//check battery level
		//enable pin connected to ADC0 
		//the ADC is not directly connected to battery because the voltage divider would draw current in standby
		PORTC |= (1 << BAT_V_CHECK_PIN); 
		int16_t accum = 0;
		for (uint8_t i = 0; i < 8; i++){
			adc_start_conversion(0);
			while(adc_irq_flag == 0){
				//usart_write_str(" W ");
				_delay_ms(2);
			}
			accum += adc_value;
			//usart_write_str(itoa(i, buff, 10));
			//usart_write_str(" ");
			//usart_write_str(itoa(adc_value, buff, 10));
			//usart_write_str("\n\r");
		}
		adc_value = accum >>3;
		adc_irq_flag = 0;
			//usart_write_str(itoa(adc_value, buff, 10));
			//usart_write_str("\n\r");
		
		PORTC &= ~(1 << BAT_V_CHECK_PIN); 
		//usart_write_str(" BAT ");
		if (adc_value < LEVEL_BAT_DISCHARGED){
			//disable stand-by	
			//usart_write_str("LOW\r");
			return BAT_DISCHARGED;
		}
		if (adc_value >= LEVEL_BAT_LOADED){
			//disable stand-by	
			//usart_write_str("FULL\r");
			return BAT_FULL;
		}
		//usart_write_str("OK\r");		
		_delay_ms(50);
		return BAT_OK;
}



int main(void)
{
	static bool b_enter_standby = true;
	static uint8_t state = true;
	setup();
	//TODO: check if the reset cause was a brown-out detection, which would mean the battery is empty
	//this can happen if the mains is not connected for a long time
	
	for (;;) { 
		
		wdt_enable(WDTO_2S);
		//enable power supply
		PORTB |= (1 << POWER_SUPPLY_ENABLE_PIN);
		
		//wait for the power supply to power up
		_delay_ms(200);
		if (int0_irq_flag > 0)
		{
			usart_write_str("MOTION ");
			state = STATE_MOTION_DETECTED;
			int0_irq_flag = 0;
			dim_up();
		}
		
		//keep light power on until timer LIGHT_ON_TIMER_SEC expires 
		//start timer 0
		start_timer0();
		state = STATE_TIMER_COUNTDOWN;
		while(counter_tick < KEEP_LIGHT_ON_DURATION){
				if (ticker) usart_write_char(ticker);
				wdt_reset();
				_delay_ms(100);
				if ( !(PIND & (1<<PD2)) ){ //PIND2 is int0  - if low, reset timer 
					counter_tick = 0;
					if (output_state != FULL_ON) dim_up();
				}
		}
		usart_write_str("Timer end");
		stop_timer0();
		b_enter_standby = true;
		counter_tick = 0;
		
		//if battery level below limit, don't disable power supply, allowing it to recharge from mains
		
		usart_write_str("\n\r");
		
		if (get_battery_state()==BAT_DISCHARGED){
			usart_write_str("Bat discharged");
			_delay_ms(100);
			wdt_reset();
			b_enter_standby = false;
		}else{
			b_enter_standby = true;
		}
		
		if (!dim_down()){
			b_enter_standby=false; //if motion detected during dim down, don't go to sleep 
			int0_irq_flag = 1; //signal motion
		}
		
		if (b_enter_standby){ //no event occured and we want to enter standby
			
			_delay_ms(100);
			state = STATE_SLEEPING;
			//disconnect power supply from mains
			PORTB &= ~(1 << POWER_SUPPLY_ENABLE_PIN); //switch off power
			usart_write_str("Sleep\n\r");
			wdt_disable();
			enable_int0_level();
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);             // select the watchdog timer mode
			sleep_enable();
			// needed??? sleep_mode();                                    // trigger the sleep
			sei();
			state = STATE_SLEEPING;
			sleep_cpu();
			/* ... program blocks here after sleep_cpu, waiting for an interrupt ... */
			sleep_disable();
			disable_int0();		
		}else{
			//int0_irq_flag = 1; //if we can't enter sleep, signal motion
			//state = STATE_MOTION_DETECTED;
		}
		
    }
}
