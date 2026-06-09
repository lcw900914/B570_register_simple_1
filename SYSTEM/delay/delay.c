#include "delay.h"

/* Include uCOS-II header if the OS is enabled */
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"
#endif

/* SysTick scale factors */
static u8  fac_us = 0;   // Ticks per microsecond
static u16 fac_ms = 0;   // Ticks per millisecond (non-OS) or ms per OS tick (OS)

#ifdef OS_CRITICAL_METHOD   // Defined only when using uCOS-II

/**************************************************************************
Function: SysTick interrupt handler - used to drive the uCOS-II time base
Input   : none
Output  : none
**************************************************************************/
void SysTick_Handler(void)
{
	OSIntEnter();    // Notify OS that an interrupt has started
	OSTimeTick();    // Update uCOS-II tick counter
	OSIntExit();     // Check if a higher-priority task should preempt
}
#endif

/**************************************************************************
Function: Initialize the delay functions.
          SysTick clock source is set to HCLK/8.
          When uCOS-II is used, also initializes the OS tick interrupt.
Input   : SYSCLK - system clock frequency in MHz
Output  : none
**************************************************************************/
void delay_init(u8 SYSCLK)
{
#ifdef OS_CRITICAL_METHOD
	u32 reload;
#endif
	SysTick->CTRL &= ~(1<<2);    // Use external clock source (HCLK/8)
	fac_us = SYSCLK / 8;         // Ticks per us = SYSCLK[MHz] / 8

#ifdef OS_CRITICAL_METHOD
	reload  = SYSCLK / 8;        // Ticks per microsecond
	reload *= 1000000 / OS_TICKS_PER_SEC;  // Reload value for one OS tick
	fac_ms  = 1000 / OS_TICKS_PER_SEC;    // Milliseconds per OS tick
	SysTick->CTRL |= 1<<1;       // Enable SysTick interrupt
	SysTick->LOAD  = reload;     // Set reload for OS tick period
	SysTick->CTRL |= 1<<0;       // Enable SysTick counter
#else
	fac_ms = (u16)fac_us * 1000; // Ticks per millisecond (no OS)
#endif
}

#ifdef OS_CRITICAL_METHOD   // uCOS-II versions

/**************************************************************************
Function: Microsecond delay (uCOS-II aware - suspends scheduler while waiting)
Input   : nus - number of microseconds to delay
Output  : none
**************************************************************************/
void delay_us(u32 nus)
{
	u32 ticks;
	u32 told, tnow, tcnt = 0;
	u32 reload = SysTick->LOAD;

	ticks = nus * fac_us;
	tcnt  = 0;
	OSSchedLock();              // Prevent task switch during short delay
	told = SysTick->VAL;
	while(1)
	{
		tnow = SysTick->VAL;
		if(tnow != told)
		{
			/* SysTick counts down; handle wrap-around */
			if(tnow < told) tcnt += told - tnow;
			else            tcnt += reload - tnow + told;
			told = tnow;
			if(tcnt >= ticks) break;
		}
	}
	OSSchedUnlock();
}

/**************************************************************************
Function: Millisecond delay (uCOS-II aware - uses OSTimeDly for long delays)
Input   : nms - number of milliseconds to delay
Output  : none
**************************************************************************/
void delay_ms(u16 nms)
{
	if(OSRunning == OS_TRUE)         // OS is running: use OS delay when possible
	{
		if(nms >= fac_ms)            // Long enough for OS tick delay
		{
			OSTimeDly(nms / fac_ms); // Delay in OS ticks
		}
		nms %= fac_ms;               // Remaining sub-tick time
	}
	delay_us((u32)(nms * 1000));     // Handle remainder with busy-wait
}

#else   /* Non-OS versions */

/**************************************************************************
Function: Microsecond delay (busy-wait using SysTick)
Input   : nus - number of microseconds to delay
Output  : none
**************************************************************************/
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = nus * fac_us;   // Set countdown value
	SysTick->VAL  = 0x00;           // Clear current value
	SysTick->CTRL = 0x01;           // Start, use external clock (HCLK/8)
	do
	{
		temp = SysTick->CTRL;
	}
	while((temp & 0x01) && !(temp & (1<<16)));  // Wait for count-to-zero flag
	SysTick->CTRL = 0x00;           // Stop SysTick
	SysTick->VAL  = 0X00;           // Clear value
}

/**************************************************************************
Function: Millisecond delay (busy-wait using SysTick)
          Maximum safe value: nms <= 0xFFFFFF * 8 * 1000 / SYSCLK_Hz
          At 72MHz: nms <= 1864
Input   : nms - number of milliseconds to delay
Output  : none
**************************************************************************/
void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms * fac_ms;  // Set countdown value
	SysTick->VAL  = 0x00;               // Clear current value
	SysTick->CTRL = 0x01;               // Start counter
	do
	{
		temp = SysTick->CTRL;
	}
	while((temp & 0x01) && !(temp & (1<<16)));  // Wait for count-to-zero flag
	SysTick->CTRL = 0x00;               // Stop SysTick
	SysTick->VAL  = 0X00;               // Clear value
}
#endif
