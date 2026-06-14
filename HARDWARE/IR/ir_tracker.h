#ifndef __IR_TRACKER_H
#define __IR_TRACKER_H
#include "sys.h"

/* ---- 3-channel IR line tracker (PD steering) ----
 * Sensors (digital DO):
 *   IR_L = PB1, IR_M = PA1, IR_R = PA2
 * Wiring: black line = LOW level (active low). Internal pull-ups enabled.
 * Logical reading: on black line = 1, on white floor = 0.
 */

/* PD gains for the steering correction. Constant-speed / gentle-steer style:
 * IR_KP kept SMALL so a turn is a mild wheel-speed difference (the bot arcs at
 * constant speed), NOT a fast in-place spin. Raise a little if it can't make a
 * corner; lower if it weaves/over-steers. */
#define IR_KP 1900   /* raised from 1400: gentle curves were sometimes under-steered and slipped off */
#define IR_KD 100

/* Hard cap on the PD steering term (PWM). Keeps the +-2 spike (~2*IR_KP) from
 * net-pushing the bot forward through the one-wheel mixing during the pivot-entry
 * window. Must stay >= IR_KP so +-1 curve steering is never clipped. */
#define IR_CORRECTION_MAX 2000

/* Fine-alignment gain used ONLY in the settle window right after a big turn:
 * a small, gentle nudge so the bot trims itself dead-centre on the line without
 * the over-shoot/jitter that the big cornering IR_KP would cause. */
#define IR_FINE_KP 400

/* Spin floor: minimum steering differential (PWM units) applied whenever the
 * bot is turning (error != 0). Must exceed the balance/velocity common-mode
 * PWM so the slower wheel goes NEGATIVE -> the two wheels counter-rotate
 * (true in-place pivot) instead of just differing in speed.
 * Bigger = more aggressive counter-rotation (may overshoot the line); 0 = off.
 * OFF for constant-speed/gentle-steer style: no forced in-place spin. */
#define IR_SPIN_MIN 0

/* ---- Big-turn in-place pivot --------------------------------------------- *
 * THE CORNER STANDARD IS ALL-WHITE (000): with only ~2cm of sensor lookahead a
 * real 90deg corner reads 000 almost immediately, so 000 (after the grace
 * window) IS the corner signal. The bot then tank-pivots in place (one wheel
 * +PIVOT_OUTER, the other an absolute -PIVOT_INNER) toward the side the line
 * was last seen, until the line reaches the centre sensor. +-2 alone never
 * pivots: it just stops and steers hard - a real corner goes 000 right after. */

/* Hard-turn in-place pivot (HARD_TURN_TANK_PIVOT): the OUTSIDE wheel drives
 * forward by PIVOT_OUTER, the INSIDE (line-side) wheel reverses by PIVOT_INNER,
 * both around common_pwm (balance term, ~0 when upright).
 *   wheel speed difference (turn rate) = PIVOT_OUTER + PIVOT_INNER
 *   PIVOT_OUTER == PIVOT_INNER -> pure in-place spin (equal-and-opposite)
 *   PIVOT_OUTER  > PIVOT_INNER -> pivots while creeping forward (arc through corner)
 *   PIVOT_OUTER  < PIVOT_INNER -> pivots while easing backward
 * Tuning:
 *   - sharp (~120 deg) corner won't turn through -> raise BOTH (more speed diff)
 *   - spins past the line / left-right oscillation -> lower BOTH
 *   - want it to keep advancing through the corner -> raise PIVOT_OUTER only
 * Each must beat ~MOTOR_DEADZONE + balance PWM for that wheel to actually drive. */
#define PIVOT_OUTER 1000   /* OUTSIDE wheel: driven at common_pwm + this (forward push, still carries the balance term) */
#define PIVOT_INNER 3000   /* INSIDE (line-side) wheel: driven at a FIXED -PIVOT_INNER (absolute reverse) - strongly dominant. Net (OUTER-INNER)/2 = -1000: rotates fast on the reverse side while easing backward, pulling the overrun corner back under the sensors */

/* Per-direction pivot strength trim (%), to even out a LEFT/RIGHT motor strength
 * mismatch. During a pivot only one motor does the forward work, so any
 * difference between the two motors shows up directly as "one direction spins
 * faster than the other" (straights hide it - the feedback loops compensate).
 * Scales BOTH the outer push and inner reverse of that direction.
 * Calibrate on the bench (wheels off the ground): trigger a 001-side pivot and a
 * 100-side pivot, compare speeds, then LOWER the faster side below 100 in steps
 * of 5-10 until both look equal. (Prefer lowering the fast side over raising the
 * slow one - less surge risk.) 100/100 = no trim. */
#define PIVOT_GAIN_ERRPOS 100  /* pivot strength % when the line was on the 001/011 side (ir_error > 0) */
#define PIVOT_GAIN_ERRNEG 100  /* pivot strength % when the line was on the 100/110 side (ir_error < 0) */

/* Forward speed target while ON the line and centred (going straight).
 * Kept LOW because the sensor lookahead is only ~2cm: the bot must be able to
 * stop within that distance the instant it detects a corner, or it overshoots
 * the vertex and loses the line. Slower = follows tighter corners.
 * 6 -> 8: the ramp is soft cardboard that SAGS (gets steeper) under the bot,
 * so it needs entry momentum to punch through before the sag develops.
 * If corners start overshooting, drop back to 6. */
#define LINE_BASE_SPEED 8



/* Gentle steering differential applied while the line is LOST, toward the LAST
 * known line side (sign of last_error). Small on purpose: it makes a soft arc
 * back toward the line, NOT an aggressive in-place spin. 0 = go dead straight. */
#define IR_LOST_CORRECTION 300

/* ---- Lost-line recovery escalation (safety) ----------------------------- *
 * While the line is lost (000), behaviour escalates with how long it's lost:
 *   cycles  < LOST_ARC_CYCLES     : phase 1, brief grace (coast)
 *   cycles  < LOST_BACKUP_CYCLES  : phase 2, REVERSE straight - only when the 000 came
 *                                   WITHOUT a pivot (blew past the corner): the vertex
 *                                   is behind the axle, no rotation can reach it
 *   cycles  < LOST_GIVEUP_CYCLES  : phase 3, pivot-search toward the last-seen side
 *                                   (PIVOT_OUTER/PIVOT_INNER), in short chunks
 *   cycles >= LOST_GIVEUP_CYCLES  : phase 4, give up -> stop searching, balance on spot
 * ~10ms per cycle. A 000 that starts DURING a pivot skips the backup (rotation is the
 * right recovery between the old and new line). */
#define LOST_ARC_CYCLES    2     /* ~20ms grace before recovery (was 6). Every grace cycle is coasting distance carried PAST the corner - the track has no line gaps to glide over, so commit to the pivot almost immediately */
#define LOST_BACKUP_CYCLES 90    /* reverse from cycle 6 to ~90 (~0.84s) to bring the overrun corner back under the sensors */
#define LINE_BACKUP_SPEED  3     /* reverse speed target while backing up (same units as LINE_BASE_SPEED) */
#define LOST_GIVEUP_CYCLES 250   /* ~2.5s total before giving up */
#define LOST_SEARCH_CHUNK  10    /* search rotates in ~100ms pivot chunks; finding the line mid-chunk overshoots at most the chunk remainder */

/* Straight-line smoothing (anti-sawtooth): when the bot returns to centre (010),
 * a small steering residual of this magnitude is seeded toward the last drift
 * side and then fades out (x3/4 per cycle), so steering eases off instead of
 * snapping to zero. 0 = off (hard switch, original behaviour). */
#define STRAIGHT_TRIM 250

/* Forward speed at a slight DRIFT (error +-1). Keep creeping forward while
 * gently steering, so small curves FLOW instead of dead-stopping and being
 * over-turned. (+-2 stops forward and steers hard in place; a real corner then
 * reads 000 and the big pivot takes over.) */
#define LINE_DRIFT_SPEED 2


/* Number of consecutive centred (010) cycles required before speeding up to
 * LINE_BASE_SPEED, applied ONLY when centre is reached directly from a lost
 * (000) state (the 000->010 re-acquire). Reaching centre from a turn speeds up
 * immediately. ~10ms per cycle, so 5 = ~50ms. */
#define CENTER_CONFIRM 10

/* Post-turn settle also requires the body to be UPRIGHT (not pitched) before
 * resuming speed: |tilt - balance setpoint| must be within this many degrees.
 * Stops the "after a turn it leans forward so the balance loop drives forward
 * and rushes off the track" failure. Bigger = resumes sooner (more lean allowed). */
#define SETTLE_ANGLE 3.0f

/* Active exit-brake cap. While still settling after a hard turn, if the bot is
 * translating forward (turn-induced lean rush) the velocity loop is commanded
 * to drive REVERSE, proportional to the measured forward speed, capped here.
 * Bigger = harder braking of the rush. 0 = off.
 * 4 backed the bot up (too strong); 1 was too weak to stop. 2 = middle: stops
 * the forward rush without driving noticeably backward. */
#define EXIT_BRAKE_MAX 8   /* raised from 3: with working encoders this is the active momentum-killer at corner entry (the old value was tuned while encoder feedback was broken and the brake never actually fired) */

/* Exit-brake DEADZONE: only brake when the forward speed (encoder sum) exceeds
 * this. Below it the bot is just balancing/wobbling in place, and braking that
 * would ratchet it slowly BACKWARD (the brake only ever pushes in reverse).
 * Raise if it still creeps backward; lower if a real rush isn't braked. */
#define EXIT_BRAKE_DEADZONE 2  /* lowered from 6: cruising speed is only ~3 (L+R counts/10ms), so a deadzone of 6 sat ABOVE cruise and the brake could never fire. 2 = brake any coasting beyond a slow walk, leave balance wobble alone */


/* Steer by pivoting around ONE wheel: when defined, the steering differential
 * is put entirely on the OUTSIDE wheel (driven forward) while the INSIDE wheel
 * is held at the balance common-mode (~still, only doing balance). Comment out
 * for the normal symmetric steering (both wheels counter-rotate around centre).
 * Turn rate per error is unchanged; the bot arcs forward instead of spinning. */
#define STEER_PIVOT_ONE_WHEEL

/* Inner-wheel steering share when STEER_PIVOT_ONE_WHEEL is enabled.
 * 0   = inner wheel gets no steering term (best for "inside wheel almost stops")
 * 50  = inner wheel gets half the normal reverse steering
 * 100 = same as normal symmetric steering
 */
#define STEER_INNER_PERCENT 50

/* Outside-wheel steering strength when STEER_PIVOT_ONE_WHEEL is enabled.
 * 100 = outside wheel gets the normal correction
 * 150 = stronger outside-wheel push
 * 200 = same wheel-to-wheel differential as normal symmetric steering
 */
#define STEER_OUTER_PERCENT 100

/* Wide track-width chassis: use true counter-rotating tank pivot for hard
 * turns, while keeping one-wheel style for small steering corrections.
 */
#define HARD_TURN_TANK_PIVOT

/* GPIO level that means "black line detected". Flip to 1 if your sensors are active-high. */
#define IR_BLACK 0

/* Master switch: 0 = pure balance (IR ignored), 1 = line following. Default OFF. */
extern u8 Flag_LineFollow;

void IR_Init(void);                       /* configure PB1/PA1/PA2 as input pull-up */
void IR_Read(u8 *l, u8 *m, u8 *r);        /* read the 3 sensors, each 0 or 1 (1 = on black line) */
int  IR_GetError(void);                   /* L/M/R -> error, holds last_error when line is lost */
int  IR_GetCorrection(void);              /* PD steering term: Kp*error + Kd*(error - prev_error) */
int  IR_GetLastError(void);              /* most recently computed error (no sensor re-read) */
u8   IR_LineLost(void);                  /* 1 = last read was all-white (000), line lost */

#endif
