#include "key.h"

/**************************************************************************
Function: Initialize the key GPIO pin (PA5, pull-up input)
Input   : none
Output  : none
**************************************************************************/
void KEY_Init(void)
{
	RCC->APB2ENR |= 1<<2;        // Enable GPIOA clock
	GPIOA->CRL &= 0XFF0FFFFF;    // Clear PA5 config
	GPIOA->CRL |= 0X00800000;    // PA5: input with pull-up/pull-down
	GPIOA->ODR |= 1<<5;          // PA5: pull-up enabled
}

/**************************************************************************
Function: Detect a single key press (edge-triggered, no debounce delay)
Input   : none
Output  : 1 if a new press is detected, 0 otherwise
**************************************************************************/
u8 click(void)
{
	static u8 flag_key = 1;  // Key release flag (1 = ready to detect)
	if(flag_key && KEY == 0) // Key pressed (PA5 pulled low)
	{
		flag_key = 0;
		return 1;            // Rising edge: new press detected
	}
	else if(1 == KEY)
		flag_key = 1;        // Key released: ready for next press
	return 0;                // No new press
}
