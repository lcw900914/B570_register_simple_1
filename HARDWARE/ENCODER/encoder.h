#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>

/* Maximum counter period for 16-bit timers on STM32F103 */
#define ENCODER_TIM_PERIOD (u16)(65535)

void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
int  Read_Encoder(u8 TIMX);
void TIM4_IRQHandler(void);
void TIM3_IRQHandler(void);

#endif
