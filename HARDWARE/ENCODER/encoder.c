#include "encoder.h"

/**************************************************************************
Function: Initialize TIM3 as encoder interface mode
Input   : none
Output  : none
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	RCC->APB1ENR |= 1<<1;       // Enable TIM3 clock
	RCC->APB2ENR |= 1<<2;       // Enable GPIOA clock
	GPIOA->CRL &= 0X00FFFFFF;   // Clear PA6, PA7 config
	GPIOA->CRL |= 0X44000000;   // PA6, PA7: floating input
	TIM3->DIER  |= 1<<0;        // Enable update interrupt (was TIM2: copy-paste bug)
	TIM3->DIER  |= 1<<6;        // Enable trigger interrupt
	MY_NVIC_Init(1, 3, TIM3_IRQn, 1);
	TIM3->PSC = 0x0;                    // No prescaler
	TIM3->ARR = ENCODER_TIM_PERIOD;     // Counter period
	TIM3->CR1 &= ~(3<<8);               // No clock division
	TIM3->CR1 &= ~(3<<5);               // Edge-aligned mode

	TIM3->CCMR1 |= 1<<0;    // CC1S='01': IC1FP1 mapped to TI1
	TIM3->CCMR1 |= 1<<8;    // CC2S='01': IC2FP2 mapped to TI2
	TIM3->CCER  &= ~(1<<1); // CC1P='0': IC1FP1 non-inverted, IC1FP1=TI1
	TIM3->CCER  &= ~(1<<5); // CC2P='0': IC2FP2 non-inverted, IC2FP2=TI2
	TIM3->CCMR1 |= 3<<4;    // IC1F='0011': input capture filter
	TIM3->SMCR  |= 3<<0;    // SMS='011': encoder mode 3, count on both edges
	TIM3->CR1   |= 0x01;    // CEN=1: enable counter
}

/**************************************************************************
Function: Initialize TIM4 as encoder interface mode
Input   : none
Output  : none
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	RCC->APB1ENR |= 1<<2;       // Enable TIM4 clock
	RCC->APB2ENR |= 1<<3;       // Enable GPIOB clock
	GPIOB->CRL &= 0X00FFFFFF;   // Clear PB6, PB7 config
	GPIOB->CRL |= 0X44000000;   // PB6, PB7: floating input
	TIM4->DIER  |= 1<<0;        // Enable update interrupt
	TIM4->DIER  |= 1<<6;        // Enable trigger interrupt
	MY_NVIC_Init(1, 3, TIM4_IRQn, 1);
	TIM4->PSC = 0x0;                    // No prescaler
	TIM4->ARR = ENCODER_TIM_PERIOD;     // Counter period
	TIM4->CR1 &= ~(3<<8);               // No clock division
	TIM4->CR1 &= ~(3<<5);               // Edge-aligned mode

	TIM4->CCMR1 |= 1<<0;    // CC1S='01': IC1FP1 mapped to TI1
	TIM4->CCMR1 |= 1<<8;    // CC2S='01': IC2FP2 mapped to TI2
	TIM4->CCER  &= ~(1<<1); // CC1P='0': IC1FP1 non-inverted, IC1FP1=TI1
	TIM4->CCER  &= ~(1<<5); // CC2P='0': IC2FP2 non-inverted, IC2FP2=TI2
	TIM4->CCMR1 |= 3<<4;    // IC1F='0011': input capture filter
	TIM4->SMCR  |= 3<<0;    // SMS='011': encoder mode 3, count on both edges
	TIM4->CR1   |= 0x01;    // CEN=1: enable counter
}

/**************************************************************************
Function: Read encoder count and reset the counter
Input   : TIMX - timer number (2, 3, or 4)
Output  : Signed encoder count since last read
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;
	switch(TIMX)
	{
		case 2:  Encoder_TIM = (short)TIM2->CNT;  TIM2->CNT = 0;  break;
		case 3:  Encoder_TIM = (short)TIM3->CNT;  TIM3->CNT = 0;  break;
		case 4:  Encoder_TIM = (short)TIM4->CNT;  TIM4->CNT = 0;  break;
		default: Encoder_TIM = 0;
	}
	return Encoder_TIM;
}

/**************************************************************************
Function: TIM4 interrupt service routine
Input   : none
Output  : none
**************************************************************************/
void TIM4_IRQHandler(void)
{
	if(TIM4->SR & 0X0001)  // Update interrupt flag
	{
	}
	TIM4->SR &= ~(1<<0);   // Clear interrupt flag
}

/**************************************************************************
Function: TIM3 interrupt service routine
Input   : none
Output  : none
**************************************************************************/
void TIM3_IRQHandler(void)
{
	if(TIM3->SR & 0X0001)  // Update interrupt flag
	{
	}
	TIM3->SR &= ~(1<<0);   // Clear interrupt flag
}
