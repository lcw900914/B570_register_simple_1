#include "ir_tracker.h"

/* Master switch - default ON so the bot line-follows from startup. Double-click
 * the key to toggle back to pure balance (see Key() in control.c). */
u8 Flag_LineFollow = 1;

/* Holds the last valid error so a lost line (000) keeps steering the same way. */
static int last_error = 0;

/* 1 when the most recent read saw all-white (000) = line lost / off the line. */
static u8 line_lost = 0;

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
          black line (active-high DO: black = pin HIGH), 0 over the white floor.
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
	static u8 cl = 0, cm = 0, cr = 0;                 // committed (debounced) sensor state
	static u8 candl = 0, candm = 0, candr = 0, candn = 0; // candidate awaiting confirmation
	u8 l, m, r;
	int error;

	/* Tilted back (Angle_Balance < IR_TRUST_MIN_ANGLE): the front-mounted IR no
	   longer faces the floor squarely, so its reading is unreliable. Don't act on
	   it - hold the last steering (same as a lost line) until the body comes back
	   up. */
	if(Angle_Balance < IR_TRUST_MIN_ANGLE)
		return last_error;

	IR_Read(&l, &m, &r);

	/* Side-sensor false-black rejection (see SIDE_BLACK_CONFIRM): bias the L/R
	 * channels toward WHITE. A side sensor must read black SIDE_BLACK_CONFIRM
	 * times in a row before we believe it; any white read clears it at once. The
	 * middle sensor is left raw so the line is tracked the instant it centres. */
	{
		static u8 lcnt = 0, rcnt = 0;
		if(l) { if(lcnt < SIDE_BLACK_CONFIRM) lcnt++; } else lcnt = 0;
		if(r) { if(rcnt < SIDE_BLACK_CONFIRM) rcnt++; } else rcnt = 0;
		l = (lcnt >= SIDE_BLACK_CONFIRM) ? 1 : 0;
		r = (rcnt >= SIDE_BLACK_CONFIRM) ? 1 : 0;
	}

	/* Debounce: a new sensor pattern must be read on TWO consecutive calls before
	 * it is acted on. A single-cycle flicker (noise, reflection, a line edge or a
	 * tiny gap) therefore can't jerk the steering around. Until a change is
	 * confirmed, keep steering by the last committed reading. ~1 cycle (~10ms) lag. */
	if(l == candl && m == candm && r == candr) { if(candn < 2) candn++; }
	else { candl = l; candm = m; candr = r; candn = 1; }
	if(candn >= 2) { cl = l; cm = m; cr = r; }        // stable for 2 reads -> commit
	l = cl; m = cm; r = cr;                            // use the debounced state from here on

	line_lost = (l==0 && m==0 && r==0) ? 1 : 0;  // all white -> line lost (drive straight to search)

	/* Graded: M alone = dead centre (go straight); M + a side = slight drift ->
	 * GENTLE steer (follows small curves without big weave); a single side with M
	 * OFF = line fully left the middle -> strong pivot (turn until centred). */
	if      (l==0 && m==1 && r==0) error =  0;   //  010 dead centre -> straight
	else if (l==0 && m==1 && r==1) error =  1;   //  011 slightly right -> gentle steer
	else if (l==0 && m==0 && r==1) error =  2;   //  001 fully right -> pivot
	else if (l==1 && m==1 && r==0) error = -1;   //  110 slightly left -> gentle steer
	else if (l==1 && m==0 && r==0) error = -2;   //  100 fully left -> pivot
	else if (l==1 && m==1 && r==1) error =  0;   //  111 intersection -> straight
	else                           error = last_error;  // 000 lost / 101 odd: hold

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

	/* Spin floor: force a minimum differential while turning so the slower
	 * wheel is driven NEGATIVE (two wheels counter-rotate), not just slowed. */
	if(error > 0 && correction <  IR_SPIN_MIN) correction =  IR_SPIN_MIN;
	if(error < 0 && correction > -IR_SPIN_MIN) correction = -IR_SPIN_MIN;

	/* Cap the steering term. At +-2 the raw PD spikes to ~2*IR_KP (~3800), and the
	 * one-wheel mixing turns that into a NET FORWARD push during the pivot-entry
	 * confirmation window (~80ms) - the "sees a corner, surges" rush. +-1 curves
	 * (1*IR_KP) stay under the cap and are unaffected. */
	if(correction >  IR_CORRECTION_MAX) correction =  IR_CORRECTION_MAX;
	if(correction < -IR_CORRECTION_MAX) correction = -IR_CORRECTION_MAX;

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

/**************************************************************************
Function: Report whether the most recent sensor read saw all-white (000),
          i.e. the line is lost / the bot is off the line.
**************************************************************************/
u8 IR_LineLost(void)
{
	return line_lost;
}
