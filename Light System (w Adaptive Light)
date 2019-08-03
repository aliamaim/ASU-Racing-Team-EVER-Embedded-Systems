/*
 * LightSystem.c
 *
 * Created: 8/2/2019 11:02:05 PM
 * Author : ali
 */ 
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
//#include "lcd.h"
#include "common_macros.h"

void ICUInit(void);
void Timer0OVFInit(void);

#define LEFTSIGNAL_IN PORTD0
#define RIGHTSIGNAL_IN PORTD1
#define WAITINGSIGNAL_IN PORTD2

#define LEFTSIGNAL_OUT PORTD3
#define RIGHTSIGNAL_OUT PORTD4


enum SignalDirection {Forward, Waiting, Left, Right};

enum SignalDirection signal_state = Forward;

int main(void)
{
	DDRD &= ~(1 << LEFTSIGNAL_IN | 1 << RIGHTSIGNAL_IN | 1 << WAITINGSIGNAL_IN);  // Set them as input
	DDRD |= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Set them as output
	DDRB |= (1 << PORTB3);
	PORTB |= (1 << PORTB3);
	ICUInit();
	Timer0OVFInit();
	sei();
    /* Replace with your application code */
    while (1) 
    {
		///////////////////// Turn Signals & Waiting ////////////////////////////////
		if(BIT_IS_SET(PORTD, WAITINGSIGNAL_IN))
		{
			if((BIT_IS_SET(PORTD, LEFTSIGNAL_OUT) && BIT_IS_SET(PORTD, RIGHTSIGNAL_OUT)) || 
			   (BIT_IS_CLEAR(PORTD, LEFTSIGNAL_OUT) && BIT_IS_CLEAR(PORTD, RIGHTSIGNAL_OUT)) )
			   {
				   PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); //Make sure both are the same state to avoid desync between the right & left signals.
			   }
			_delay_ms(25); // Debounce time
			if(BIT_IS_SET(PORTD, WAITINGSIGNAL_IN)) { signal_state = Waiting; }
		}
		else if(BIT_IS_SET(PORTD, LEFTSIGNAL_IN))
		{
			_delay_ms(25); // Debounce time
			if(BIT_IS_SET(PORTD, LEFTSIGNAL_IN)) { signal_state = Left; }
		}
		else if(BIT_IS_SET(PORTD, RIGHTSIGNAL_IN))
		{
			_delay_ms(25); // Debounce time
			if(BIT_IS_SET(PORTD, RIGHTSIGNAL_IN)) { signal_state = Right; }
		}
		else
		{
			signal_state = Forward;
		}
		///////////////////////////////////////////////////////////////////////////
		
		
		/////////////////////////// Headlights ////////////////////////////////////
		
		///////////////////////////////////////////////////////////////////////////
		
		
		
		/////////////////////////// Brakes lights ///////////////////////////////////
		
		////////////////////////////////////////////////////////////////////////////
    }
}

void ICUInit(void)
{
	DDRB &= ~(1 << PORTB0); //Set PB0 (ICP1) as I/P
	TCCR1B = 0;
	TCCR1B = (1 << ICES1) | (1 << CS12) | (1 << CS10);
	/*
	* WGM13 = 0, WGM12 = 0.  Normal Mode
	* ICNC1 = 0. Disable Input Capture Noise Filter
	* ICES1 = 1. Initially set the input capture at rising edge (will be toggled at each capture)
	* CS12 CS11 CS10 (1 0 1), set clock to CPU_CLOCK/1024.
	*/
	TIFR1 |= (1<<ICF1); //Clear input capture flag
	TIMSK1 |= (1 << ICIE1); //Enable interrupts on Input Capture for timer1
}

void Timer0OVFInit(void)
{
	//Set timer to normal mode (overflow mode)
	TCCR0B &= ~(1 << WGM00 | 1 << WGM01);
	TCCR0A &= ~(1 << WGM02);
	// set up timer with prescaler = 1024
	TCCR0B |= (1 << CS02)|(1 << CS00);
	// initialize counter
	TCNT0 = 0;
}


ISR(TIMER0_OVF_vect)
{
	if(signal_state == Waiting)
	{
		PORTD ^= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT);
	}
	else if(signal_state == Left)
	{
		TOGGLE_BIT(PORTD, LEFTSIGNAL_OUT);
	}
	else if(signal_state == Right)
	{
		TOGGLE_BIT(PORTD, RIGHTSIGNAL_OUT);
	}
	else
	{
		PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Turn off the signals
		TIMSK0 &= ~(1 << TOIE0); // Disable timer interrupt so the ISR that toggles the turn signals doesn't get called
		TCNT0 = 0; // Clear timer counter
	}
	
}

ISR(TIMER1_CAPT_vect)
{
	// Enable the timer0 OVF interrupt which would control the signals.
	TIMSK0 |= (1 << TOIE0);
	// Call the timer0 which controls the turning signals for the first time so the driver would see immediate change (not neccessary)
	//TIMER0_OVF_vect();
}


