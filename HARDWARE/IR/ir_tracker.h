#ifndef __IR_TRACKER_H
#define __IR_TRACKER_H
#include "sys.h"

/* ---- 3-channel IR line tracker (PD steering) ----
 * Sensors (digital DO):
 *   IR_L = PB1, IR_M = PA1, IR_R = PA2
 * Wiring: black line = HIGH level (active high, this board's DO output).
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
#define PIVOT_OUTER 850   /* OUTSIDE wheel: driven at common_pwm + this. Lowered to 800 for a SLOW in-place turn so a big corner doesn't get overshot/thrown off the track. Raise both if it stalls/turns too slowly */
#define PIVOT_INNER 1250   /* INSIDE (line-side) wheel: FIXED -PIVOT_INNER (absolute reverse). EQUAL to PIVOT_OUTER -> pure in-place spin (slow): one wheel +800, the other -800, no net forward/backward creep. If it OVERSHOOTS the corner, raise PIVOT_INNER a little above PIVOT_OUTER; too slow -> raise BOTH together */

/* How many control cycles (10ms each) one corner-pivot chunk lasts before it
 * re-checks the sensors. Small -> stops close to centre (little overshoot) but
 * re-evaluates often; the +-2 corner handler re-arms it every cycle the line is
 * still off-centre, so the pivot continues until the line re-centres. */
#define CORNER_PIVOT_CHUNK 3

/* After a big-turn pivot finishes, drive STRAIGHT FORWARD for a short window
 * (no steering, no pivot) so the bot settles onto the new line instead of
 * immediately re-reacting and ping-ponging. CYCLES are 10ms ticks (30 = ~0.3s);
 * SPEED is the forward target (same units as LINE_BASE_SPEED). One per corner
 * (gated by the lock). Set CYCLES 0 to disable. */
#define POST_PIVOT_STRAIGHT_CYCLES 90   /* straight-forward window after a turn */
#define POST_PIVOT_STRAIGHT_SPEED  3    /* forward speed during the straight commit */

/* Minimum turn length (cycles, 10ms each) before a backup is allowed: the bot
 * must have been TURNING (off-centre / pivoting) continuously for at least this
 * many cycles when it returns to centre, or no backup fires. Filters out small
 * +-1 wobbles and brief corrections so ONLY a real corner backs up - this is
 * what stops the pivot<->backup ping-pong. Bigger = only sharper/longer turns
 * back up; smaller = even gentle bends back up. */
#define POST_PIVOT_MIN_TURN 15

/* Backup re-arm: after one backup fires it LOCKS, and only unlocks once the bot
 * has run cleanly centred (010, not pivoting) for this many consecutive cycles -
 * i.e. it has genuinely left the corner. Keep it long enough that the brief
 * centre-touches during a tight-corner struggle never add up to it, so each
 * corner backs up at most once. Bigger = stricter (must settle longer). */
#define POST_PIVOT_REARM 30

/* Brake-before-pivot (stop AT the corner instead of charging past it):
 *   CORNER_BRAKE_STOP  : encoder-sum (L+R per 10ms) below which the bot counts as
 *                        "stopped enough" to start the in-place pivot. Bigger =
 *                        pivots while still creeping (less braking); smaller =
 *                        insists on a fuller stop first (more anti-overshoot).
 *   CORNER_BRAKE_SPEED : reverse target commanded while braking, to actively kill
 *                        forward momentum (same units as LINE_BASE_SPEED). Bigger
 *                        = harder/faster stop (may ease backward a touch). */
#define CORNER_BRAKE_STOP  2
#define CORNER_BRAKE_SPEED 4

/* After the forward stop, reverse for this many cycles (10ms each) before the
 * pivot starts, to pull the corner back under the sensors. 0 = just stop, no
 * extra reverse. Bigger = backs up more before turning. */
#define CORNER_BACKUP_CYCLES 20

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
#define LINE_BASE_SPEED 2



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
#define LINE_DRIFT_SPEED 1


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
 * SET TO 100 (原地轉): the inside wheel now reverses by the SAME amount the
 * outside wheel drives forward, so a steering correction is a true in-place
 * rotation (一前一後速度相同) instead of a wide forward arc. Slow forward creep
 * still comes from LINE_DRIFT_SPEED / the post-turn resume. Lower back toward 50
 * if slight (+-1) drifts make it weave/oscillate on the straight.
 */
#define STEER_INNER_PERCENT 100

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

/* GPIO level that means "black line detected".
 * These sensors are ACTIVE-LOW: over a BLACK line the DO pin reads 0 (low),
 * over the WHITE floor it reads 1 (high). So IR_BLACK = 0 (matches the
 * "可以平衡" build). Flip to 1 if you swap to active-high sensors. */
#define IR_BLACK 0

/* IR-trust gate: when Angle_Balance is below this (degrees), the bot is pitched
 * back and the front IR doesn't face the floor squarely -> its reading is
 * ignored (IR_GetError holds the last steering). 0.0 = distrust whenever the
 * angle goes negative; lower it (e.g. -3) to tolerate a small backward lean
 * before distrusting, raise toward 0/positive to distrust more eagerly. */
#define IR_TRUST_MIN_ANGLE 0.0f

/* ---- Side-sensor false-black rejection (側黑線門檻) ---------------------- *
 * The L/R side sensors sometimes read BLACK over the white floor (reflection,
 * floor texture, the digital threshold sitting right at the margin) -> a phantom
 * 110/011 that jerks the steering. This is a SOFTWARE threshold on TOP of the
 * sensor's own pot: a side sensor must report black for SIDE_BLACK_CONFIRM
 * consecutive reads (~10ms each) before it is believed; a white read clears it
 * instantly. So the side channels are biased toward white and a brief false
 * black can't trigger a turn. The MIDDLE sensor is NOT filtered (the line must
 * always be tracked the instant it reaches centre).
 *   bigger  = more white-biased, ignores longer false-black blips (but a real
 *             side line must persist longer before it steers)
 *   1       = off (believe a side sensor immediately, original behaviour) */
#define SIDE_BLACK_CONFIRM 3

/* ---- Descent steering trim (下坡往右 補償) ------------------------------- *
 * While riding the tipped board DOWN the bot is in pure balance, so both wheels
 * get identical PWM and any left/right MOTOR strength mismatch shows up as a
 * consistent veer (here: it drifts RIGHT). This injects a small fixed steering
 * term during the descent window only, to cancel that drift.
 *   NEGATIVE = steer LEFT  (use this to counter a rightward veer)
 *   POSITIVE = steer RIGHT (use if it ever veers left)
 *   0        = off (no trim)
 * Tune in steps of ~100: make it more negative if it still pulls right, less if
 * it now over-corrects to the left. Only active during descent (mode = DOWN). */
#define DESCENT_STEER_TRIM (-300)

/* Master switch: 0 = pure balance (IR ignored), 1 = line following. Default OFF. */
extern u8 Flag_LineFollow;

void IR_Init(void);                       /* configure PB1/PA1/PA2 as input pull-up */
void IR_Read(u8 *l, u8 *m, u8 *r);        /* read the 3 sensors, each 0 or 1 (1 = on black line) */
int  IR_GetError(void);                   /* L/M/R -> error, holds last_error when line is lost */
int  IR_GetCorrection(void);              /* PD steering term: Kp*error + Kd*(error - prev_error) */
int  IR_GetLastError(void);              /* most recently computed error (no sensor re-read) */
u8   IR_LineLost(void);                  /* 1 = last read was all-white (000), line lost */

#endif
