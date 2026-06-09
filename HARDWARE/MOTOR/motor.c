#include "motor.h"

// Init direction pins PB12~PB15 as push-pull output
void MiniBalance_Motor_Init(void)
{
    RCC->APB2ENR |= 1<<3;      // Enable GPIOB clock
    GPIOB->CRH &= 0X0000FFFF;  // Clear PB12~PB15 config
    GPIOB->CRH |= 0X22220000;  // PB12~PB15: output push-pull, 2MHz
}

// Init TIM1 PWM on PA8 (PWMA, CH1) and PA11 (PWMB, CH4)
// freq = 72MHz / (psc+1) / (arr+1)
// e.g. MiniBalance_PWM_Init(7199, 0) -> 10kHz
void MiniBalance_PWM_Init(u16 arr, u16 psc)
{
    MiniBalance_Motor_Init();

    RCC->APB2ENR |= 1<<11;      // Enable TIM1 clock
    RCC->APB2ENR |= 1<<2;       // Enable GPIOA clock
    GPIOA->CRH &= 0XFFFF0FF0;   // Clear PA8, PA11 config
    GPIOA->CRH |= 0X0000B00B;   // PA8, PA11: AF push-pull output

    TIM1->ARR = arr;             // Auto-reload value (period)
    TIM1->PSC = psc;             // Prescaler

    TIM1->CCMR1 |= 6<<4;        // CH1 PWM mode 1
    TIM1->CCMR2 |= 6<<12;       // CH4 PWM mode 1
    TIM1->CCMR1 |= 1<<3;        // CH1 preload enable
    TIM1->CCMR2 |= 1<<11;       // CH4 preload enable
    TIM1->CCER  |= 1<<0;        // CH1 output enable
    TIM1->CCER  |= 1<<12;       // CH4 output enable
    TIM1->BDTR  |= 1<<15;       // MOE: main output enable (required for TIM1)
    TIM1->CR1    = 0x80;        // ARPE enable
    TIM1->CR1   |= 0x01;        // Start timer
}
