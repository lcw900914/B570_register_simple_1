#include "usart.h"

/* Redirect printf() to USART1 (requires "Use MicroLIB" in Keil project settings) */
#if 1
#pragma import(__use_no_semihosting)

/* Minimal FILE structure required by the standard library */
struct __FILE
{
	int handle;
};
FILE __stdout;

/* Stub to prevent semihosting exit */
_sys_exit(int x)
{
	x = x;
}

/* Redirect fputc so printf sends characters to USART1 */
int fputc(int ch, FILE *f)
{
	while((USART1->SR & 0X40) == 0);  // Wait until transmit buffer is empty
	USART1->DR = (u8)ch;
	return ch;
}
#endif

/**************************************************************************
Function: Send one byte via USART1 (blocking)
Input   : data - byte to transmit
Output  : none
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR & 0x40) == 0);  // Wait for transmission complete
}

/**************************************************************************
Function: Initialize USART1 (PA9=TX, PA10=RX), 1 stop bit, no parity
Input   : pclk2 - APB2 clock frequency in MHz
          bound - desired baud rate (e.g. 115200)
Output  : none
**************************************************************************/
void uart_init(u32 pclk2, u32 bound)
{
	float temp;
	u16   mantissa;
	u16   fraction;

	/* Calculate BRR value: USARTDIV = fCK / (16 * baud) */
	temp      = (float)(pclk2 * 1000000) / (bound * 16);
	mantissa  = temp;                      // Integer part
	fraction  = (temp - mantissa) * 16;    // Fractional part (4 bits)
	mantissa <<= 4;
	mantissa += fraction;

	RCC->APB2ENR |= 1<<2;    // Enable GPIOA clock
	RCC->APB2ENR |= 1<<14;   // Enable USART1 clock

	/* PA9: AF push-pull (TX); PA10: floating input (RX) */
	GPIOA->CRH &= 0XFFFFF00F;
	GPIOA->CRH |= 0X000008B0;

	/* Reset USART1 */
	RCC->APB2RSTR |=  1<<14;
	RCC->APB2RSTR &= ~(1<<14);

	/* Configure baud rate and enable TX+RX */
	USART1->BRR  = mantissa;   // Set baud rate
	USART1->CR1 |= 0X200C;     // UE=1 (enable), TE=1 (TX enable), RE=1 (RX enable), 1 stop bit, no parity
}
