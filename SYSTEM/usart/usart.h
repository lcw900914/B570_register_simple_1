#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"

void usart1_send(u8 data);                 // Send one byte via USART1
void uart_init(u32 pclk2, u32 bound);     // Initialize USART1 (PA9=TX, PA10=RX)

#endif
