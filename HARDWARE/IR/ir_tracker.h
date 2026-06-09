#ifndef __IR_TRACKER_H
#define __IR_TRACKER_H
#include "sys.h"

/* ---- 3-channel IR line tracker (PD steering) ----
 * Sensors (digital DO):
 *   IR_L = PB1, IR_M = PA1, IR_R = PA2
 * Wiring: black line = LOW level (active low). Internal pull-ups enabled.
 * Logical reading: on black line = 1, on white floor = 0.
 */

/* PD gains for the steering correction (tune later). */
#define IR_KP 700
#define IR_KD 140

/* Forward speed target used while line-following (velocity-loop units).
 * 0 = steer in place only (safe default). Raise to actually travel the line. */
#define LINE_BASE_SPEED 1

/* GPIO level that means "black line detected". Flip to 1 if your sensors are active-high. */
#define IR_BLACK 0

/* Master switch: 0 = pure balance (IR ignored), 1 = line following. Default OFF. */
extern u8 Flag_LineFollow;

void IR_Init(void);                       /* configure PB1/PA1/PA2 as input pull-up */
void IR_Read(u8 *l, u8 *m, u8 *r);        /* read the 3 sensors, each 0 or 1 (1 = on black line) */
int  IR_GetError(void);                   /* L/M/R -> error, holds last_error when line is lost */
int  IR_GetCorrection(void);              /* PD steering term: Kp*error + Kd*(error - prev_error) */
int  IR_GetLastError(void);              /* most recently computed error (no sensor re-read) */

#endif
