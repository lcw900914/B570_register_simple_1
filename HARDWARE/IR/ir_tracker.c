#include "ir_tracker.h"

/* Master switch - safe default OFF so this module has zero effect on balance
 * until you deliberately enable it. */
u8 Flag_LineFollow = 0;

/* Holds the last valid error so a lost line (000) keeps steering the same way. */
static int last_error = 0;

/**************************************************************************
Function: Configure the three IR sensor pins as input with pull-up.
          PB1 (IR_L), PA1 (IR_M), PA2 (IR_R). Does not touch any other pin.
**************************************************************************/
void IR_Init(void)
{
	RCC->APB2ENR |= 1<<2;          // Enable GPIOA clock
	RCC->APB2ENR |= 1<<3;          // Enable GPIOB clock

	/* PA1 (bits 7:4) and PA2 (bits 11:8): input with pull-up/pull-down */
	GPIOA->CRL &= 0xFFFFF00F;      // clear only PA1 & PA2 config nibbles
	GPIOA->CRL |= 0x00000880;      // PA1=0x8, PA2=0x8 -> input with pull
	GPIOA->ODR |= (1<<1) | (1<<2); // ODR=1 -> pull-UP on PA1, PA2

	/* PB1 (bits 7:4): input with pull-up/pull-down */
	GPIOB->CRL &= 0xFFFFFF0F;      // clear only PB1 config nibble
	GPIOB->CRL |= 0x00000080;      // PB1=0x8 -> input with pull
	GPIOB->ODR |= (1<<1);          // ODR=1 -> pull-UP on PB1
}

/**************************************************************************
Function: Read the three IR sensors. Output is 1 when the sensor is over the
          black line (active-low DO), 0 over the white floor.
**************************************************************************/
void IR_Read(u8 *l, u8 *m, u8 *r)
{
	*l = (PBin(1) == IR_BLACK) ? 1 : 0;   // IR_L = PB1
	*m = (PAin(1) == IR_BLACK) ? 1 : 0;   // IR_M = PA1
	*r = (PAin(2) == IR_BLACK) ? 1 : 0;   // IR_R = PA2
}

/**************************************************************************
Function: Map the L/M/R pattern to a position error. Positive = line is to
          the right, negative = to the left. Lost line (000) holds last_error.
**************************************************************************/
int IR_GetError(void)
{
	u8 l, m, r;
	int error;
	IR_Read(&l, &m, &r);

	if      (l==0 && m==1 && r==0) error =  0;   //  on center
	else if (l==0 && m==1 && r==1) error =  1;   //  drifting left of line
	else if (l==0 && m==0 && r==1) error =  2;   //  hard right
	else if (l==1 && m==1 && r==0) error = -1;   //  drifting right of line
	else if (l==1 && m==0 && r==0) error = -2;   //  hard left
	else if (l==1 && m==1 && r==1) error =  0;   //  intersection / all black
	else                           error = last_error;  // 000 (lost) or odd combo: hold

	last_error = error;
	return error;
}

/**************************************************************************
Function: PD steering correction. correction = Kp*error + Kd*(error - prev).
**************************************************************************/
int IR_GetCorrection(void)
{
	static int prev_error = 0;
	int error = IR_GetError();
	int correction = IR_KP * error + IR_KD * (error - prev_error);
	prev_error = error;
	return correction;
}

/**************************************************************************
Function: Return the most recently computed error without re-reading the
          sensors (used by the control loop for the turn-slowdown test).
**************************************************************************/
int IR_GetLastError(void)
{
	return last_error;
}
