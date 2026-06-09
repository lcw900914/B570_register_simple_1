#ifndef __DELAY_H
#define __DELAY_H
#include "sys.h"

void delay_init(u8 SYSCLK);   // Initialize SysTick for delay functions
void delay_ms(u16 nms);        // Delay in milliseconds
void delay_us(u32 nus);        // Delay in microseconds

#endif
