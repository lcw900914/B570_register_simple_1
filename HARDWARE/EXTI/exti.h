#ifndef __EXTI_H
#define __EXIT_H
#include "sys.h"

/* INT is connected to PA0, which receives the interrupt signal from MPU6050
   (moved from PA12, which was electrically dead). */
#define INT PAin(0)

void EXTI_Init(void);   // External interrupt initialization

#endif
