#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

/* KEY is connected to PA5, active low */
#define KEY PAin(5)

void KEY_Init(void);   // Key GPIO initialization
u8   click(void);      // Single-click detection (returns 1 on press)

#endif
