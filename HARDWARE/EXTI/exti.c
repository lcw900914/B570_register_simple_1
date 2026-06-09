#include "exti.h"

/**************************************************************************
Function: Initialize external interrupt on PA12 (falling edge, from MPU6050)
Input   : none
Output  : none
**************************************************************************/
void EXTI_Init(void)
{
	RCC->APB2ENR |= 1<<2;         // Enable GPIOA clock
	GPIOA->CRL &= 0XFFFFFFF0;    // Clear PA0 config
	GPIOA->CRL |= 0X00000008;    // PA0: input with pull-up/pull-down
	GPIOA->ODR |= 1<<0;           // PA0: pull-up enabled
	Ex_NVIC_Config(GPIO_A, 0, FTIR);            // Falling-edge trigger on PA0 (MPU INT moved here from broken PA12)
	MY_NVIC_Init(2, 1, EXTI0_IRQn, 2);         // Preemption priority 2, sub-priority 1, group 2
}
