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

void PCINT0Init(void);
void INT0Init(void);
void Timer0OVFInit(void);

#define LEFTSIGNAL_IN PORTB0
#define RIGHTSIGNAL_IN PORTB1
#define WAITINGSIGNAL_IN PORTB2
#define BRAKES_IN PORTD2 // It can only be PORTD2(INT0) or PORTD3(INT1)

#define LEFTSIGNAL_OUT PORTD5
#define RIGHTSIGNAL_OUT PORTD6
#define BRAKES_OUT PORTD0


enum SignalDirection {Forward, Waiting, Left, Right};
	
volatile enum SignalDirection signal_state = Forward;
volatile uint8_t portbhistory_signals = 0xFF;     // default is high because the pull-up
volatile uint8_t brakes_state = 0; // 0 is low, 1 is high


int main(void)
{
	sei();
	DDRD |= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT | 1 << BRAKES_OUT); // Set them as output
	PCINT0Init();
	INT0Init();
	Timer0OVFInit();
    /* Replace with your application code */
    while (1) 
    {
    }
}


void PCINT0Init(void)
{
	DDRB &= ~(1 << LEFTSIGNAL_IN | 1 << RIGHTSIGNAL_IN | 1 << WAITINGSIGNAL_IN);  // Set them as input
	PORTB |= (1 << LEFTSIGNAL_IN | 1 << RIGHTSIGNAL_IN | 1 << WAITINGSIGNAL_IN); // Turn internal pull up resistors on
	PCICR |= (1 << PCIE0); // Enable Pin Change scan on PCINT[7:0]
	PCMSK0 |= (1 << PCINT0 | 1 << PCINT1 | 1 << PCINT2); // Enable pin change interrupt for the individual pins (they all call the same ISR)
	// PCINT0 --> LEFTSIGNAL_IN
	// PINCT1 --> RIGHTSIGNAL_IN
	// PCINT2 --> WAITING_IN
	
}

void INT0Init(void)
{
	DDRD &= ~(1 << PORTD2);  // Set it as I/P
	PORTD |= (1 << PORTD2);  // Turn on pull up resistor
	EICRA |= (1 << ISC00);   // Any logical change on INT0 generates an interrupt request.
	EICRA &= ~(1 << ISC01);
	EIMSK |= (1 << INT0);
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
		if(BIT_IS_SET(PIND, LEFTSIGNAL_OUT) != BIT_IS_SET(PIND, RIGHTSIGNAL_OUT))
		{
			PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Make sure both are the same state to avoid desync between the right & left signals.
		}
		PORTD ^= (1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT);
	}
	else if(signal_state == Left)
	{
		CLEAR_BIT(PORTD, RIGHTSIGNAL_OUT);
		TOGGLE_BIT(PORTD, LEFTSIGNAL_OUT);
	}
	else if(signal_state == Right)
	{
		CLEAR_BIT(PORTD, LEFTSIGNAL_OUT);
		TOGGLE_BIT(PORTD, RIGHTSIGNAL_OUT);
	}
	else
	{
		PORTD &= ~(1 << LEFTSIGNAL_OUT | 1 << RIGHTSIGNAL_OUT); // Turn off the signals
		TIMSK0 &= ~(1 << TOIE0); // Disable timer interrupt so the ISR that toggles the turn signals doesn't get called
		TCNT0 = 0; // Clear timer counter
	}
	
}

ISR(PCINT0_vect)
{
	uint8_t changedbits;
	
	changedbits = PINB ^ portbhistory_signals;
	portbhistory_signals = PINB;
	if(changedbits & (1 << WAITINGSIGNAL_IN))
	{
		if(BIT_IS_CLEAR(PINB, WAITINGSIGNAL_IN))
			signal_state = Waiting;
		else
			signal_state = Forward;
	}
	else if(changedbits & (1 << LEFTSIGNAL_IN))
	{
		if(BIT_IS_CLEAR(PINB, LEFTSIGNAL_IN))
			signal_state = Left;
		else
			signal_state = Forward;
	}
	else if(changedbits & (1 << RIGHTSIGNAL_IN))
	{
		if(BIT_IS_CLEAR(PINB, RIGHTSIGNAL_IN))
			signal_state = Right;
		else
			signal_state = Forward;
	}
	else
	{
		signal_state = Forward;
	}
	TIMSK0 |= (1 << TOIE0);
}

ISR (INT0_vect)
{
	if(brakes_state == 0)
	{
		PORTD |= (1 << BRAKES_OUT);
		brakes_state = 1;
	}
	else
	{
		PORTD &= ~(1 << BRAKES_OUT);
		brakes_state = 0;
	}
}






