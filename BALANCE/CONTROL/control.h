#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

/* Mechanical equilibrium angle / setpoint (degrees) - applied MODE-DEPENDENTLY:
 *   Middle_angle        : used while balancing / on a slope / descending (the value
 *                         the bot stands and climbs well at).
 *   MIDDLE_ANGLE_TRACK  : used while FLAT line-following. A touch less forward-biased
 *                         so the bot does not run away forward while tracking (this is
 *                         the setpoint the working 循跡 build used).
 * The active value is selected each cycle in EXTI0_IRQHandler (see Track_Mode). */
#define Middle_angle        9.0f
#define MIDDLE_ANGLE_TRACK  9.0f   /* TEST: =Middle_angle, so the setpoint no longer switches (was 7.1). If the forward surge stops, the 7.1 line-follow setpoint was the cause and 9.0 is this bot's true balance point. */

/* Fine trim (degrees) added on top of the runtime-calibrated Balance_Target.
 * The jig may hold the bot slightly off the true balance point, leaving an
 * asymmetric "dashes/falls forward but recovers backward" behaviour.
 * Tune by binary search: dashes/falls FORWARD -> make MORE NEGATIVE;
 * dashes/falls BACKWARD -> more positive. Steps ~0.5 deg.
 * (0.5 and 1.0 were tried against the backward creep with NO effect - the creep
 * turned out to be slope-induced, not a setpoint error; fixed instead by raising
 * the velocity-integral clamp in control.c so the bot has real sustained push.) */
#define BALANCE_TRIM (0)

/* Motor dead-zone compensation (PWM units, full scale = 7199).
 * The geared motors won't turn below ~600-1000 PWM, so small tilt errors
 * produce no motion until the robot leans far over. This offset is added to
 * every non-zero control output so the motor reacts to small tilts too.
 * Tune: raise if it still won't move at small angles; lower if it buzzes /
 * twitches / oscillates while standing near upright. Set to 0 to disable. */
#define MOTOR_DEADZONE 350

/* ===== Forward-motion tuning (balance WHILE moving) ===================== */

/* Max change of the forward-speed target per 10ms control tick.
 * A front-heavy bot pitches/lurches on sudden speed changes, so we ramp the
 * target gently instead of stepping it. Bigger = snappier accel (more nod);
 * smaller = smoother but laggier. 1 = climb one speed unit per tick (1,2,3,4...). */
#define MOVE_RAMP_STEP 1

/* Forward-lean feed-forward: degrees of balance-setpoint tilt per unit of
 * (ramped) forward-speed target. Leaning the body INTO the travel direction
 * lets the bot move without the velocity loop pushing the wheels out from
 * under it - which is exactly what makes a front-heavy bot "run away".
 *   0.0f = disabled (original pure parallel-velocity behaviour).
 * Enable with a SMALL value (e.g. 0.01f). If commanding "forward" makes the
 * bot accelerate BACKWARD, flip the sign. Tune up until forward motion needs
 * little velocity-loop effort, but not so far that it pitches over. */
#define FORWARD_LEAN_GAIN 0.0f

/* Hard cap on the feed-forward lean (degrees) so a big speed command can't
 * tilt the setpoint past what Balance_Kd can still damp. */
#define FORWARD_LEAN_MAX 3.0f

/* Speed-loop integral gain - applied MODE-DEPENDENTLY in Velocity():
 *   VELOCITY_KI       : while STANDING (Move_Target == 0). Keep SMALL - it is the
 *                       standstill position hold, and a large value here winds up
 *                       and dashes the bot on a push. -0.4 is the value that balanced
 *                       cleanly under DEBUG.
 *   VELOCITY_KI_CLIMB : while DRIVING / CLIMBING (Move_Target != 0). This is the
 *                       sustained UPHILL push - make it strong. -1.5 climbed well
 *                       before; the integral clamp (+-4000) x this is the max push.
 *                       If the climb still lacks force, make it MORE negative (-1.8/
 *                       -2.0) or raise the integral clamp in control.c. If flat
 *                       driving hunts/launches, back off toward -1.0. */
#define VELOCITY_KI       -0.4f
#define VELOCITY_KI_CLIMB -1.5f

/* Speed-loop proportional gain (damping) - applied MODE-DEPENDENTLY (see Track_Mode):
 *   VELOCITY_KP_TRACK : while FLAT line-following (the proven 循跡 value).
 *   VELOCITY_KP_BAL   : while balancing / on a slope / descending (more damping). */
#define VELOCITY_KP_TRACK (-160)
#define VELOCITY_KP_BAL   (-200)

/* ===== Seesaw (蹺蹺板) tip-over handling ================================ */
/* Tip detection by a SHARP PITCH DROP from the climb peak (Angle_Balance = OLED
 * Angle). While line-following:
 *   ARM  : Angle_Balance climbs ABOVE SEESAW_CLIMB_PITCH and holds it for
 *          SEESAW_CLIMB_CONFIRM cycles; the climb high-water mark (climb_peak) is
 *          then tracked.
 *   TRIP : once armed, the pitch FALLS more than SEESAW_TIP_DROP below climb_peak
 *          (the board flips and the angle plunges, e.g. ~20 -> -5) -> line-follow
 *          OFF, dump the climb push, ride the new downhill in pure balance.
 * Because the trip is a DROP relative to the actual climb height, it adapts to the
 * slope and normal climb wobble (far smaller than the drop) cannot false-fire. */
#define SEESAW_CLIMB_PITCH   14.0f  /* deg of forward pitch (Angle_Balance) that arms the detector. MUST sit ABOVE the lean of fast flat line-following (~10-12 deg while driving) but BELOW the real climb peak (~17-20). At 10 it false-armed on the flat: a hard turn/decel then swung the angle back and TRIPPED the seesaw logic mid-track -> dropped to balance mode -> fell. 14 separates flat line-following from a genuine climb by angle (no manual mode switch needed). Raise toward 16 if flat still false-arms; lower if a real climb fails to arm. */
#define SEESAW_CLIMB_CONFIRM 60     /* x10ms the climb pitch must persist to ARM ("uphill confirmed" switch) = 0.6s. This is the gate that stops the seesaw trip from firing during flat line-following: a real climb holds >SEESAW_CLIMB_PITCH for seconds, but a line-follow start-lurch / hard accel / turn only pitches up for a fraction of a second, so it never reaches 0.6s and never arms. Raise (80/100) if line-following still arms it; lower if a genuine climb is too brief to arm. */
/* ---- Climb slowdown (after arming) ----
 * Once the detector has ARMED (a full-speed climb already reached SEESAW_CLIMB_PITCH
 * and latched), the forward speed command is capped to SEESAW_CLIMB_SPEED for the
 * run-in to the tip. A slower approach carries LESS forward momentum and winds up
 * LESS speed-integral, so when the seesaw tips the forward dive is gentler.
 * IMPORTANT: the slowdown is gated on the ARM LATCH, not on pitch. An earlier
 * pitch-gated version slowed the bot before it reached the arm angle, robbed the
 * climb push, so the pitch never hit 17 and it NEVER armed / switched to balance.
 * Flat ground and the initial climb (pre-arm) keep full LINE_BASE_SPEED. */
#define SEESAW_CLIMB_SPEED      6      /* capped forward speed once armed (same units as LINE_BASE_SPEED=8). Raised 4 -> 6 for more climb drive/momentum. Raise to 8 (= no slowdown) for max; lower if the extra momentum makes the tip too violent */
/* The tip is detected as a SHARP DROP of the pitch from the climb high-water mark
 * (climb_peak), not a fixed angle: when the board flips, the pitch plunges (e.g.
 * ~20 -> -5). This auto-adapts to whatever angle the climb actually reached.
 *   - (climb_peak - Angle_Balance) > SEESAW_TIP_DROP : the pitch collapsed -> TIPPED
 *   - Angle_Balance > SEESAW_TIP_FWD_PITCH           : (backup) dove FORWARD past the climb */
#define SEESAW_TIP_DROP      20.0f  /* deg the pitch must FALL below its climb peak to count as a tip. Peak ~20 -> trips around 0 (set 25 to require the full ~20 -> -5 drop; lower to ~15 to catch the tip sooner). Must exceed normal climb wobble so steady climbing doesn't trip */
#define SEESAW_TIP_FALL_TO   3.0f   /* deg: the tip ALSO requires the pitch to have actually fallen BELOW this (clearly tipped toward downhill), not just dropped from a transient peak. A flexing board can briefly spike the angle high (inflating climb_peak) then return to a still-CLIMBING angle (~15); without this guard that looked like a big drop and false-tripped mid-climb. The real tip plunges to ~-5, well under this. Raise toward the climb angle to trip earlier; lower (toward 0/negative) to require a more definite tip */
#define SEESAW_TIP_FWD_PITCH (25.0f)/* deg: forward-side backup trip (a forward dive raises the angle, which the peak-drop can't see). Climb peaks ~17-20, so keep clearly above (25). Lower toward 22 to catch sooner; raise if a steep climb peak false-trips */
/* ---- Early anti-dive (acts BEFORE the drop reaches SEESAW_TIP_DROP) ----
 * The peak-drop trip needs the angle to fall a long way; while it is still falling
 * the wheels can already start slamming forward. So the moment an armed climb dips
 * below SEESAW_BRAKE_PITCH we pre-empt the dive: dump the climb integral and clamp
 * forward drive, even though the full drop threshold hasn't been reached yet. */
#define SEESAW_BRAKE_PITCH   3.0f   /* deg: once armed, Angle_Balance falling BELOW this starts the anti-dive (dump push + clamp forward). MUST sit below the upright setpoint (~7) so plain standing / a hump crest that settles upright does NOT engage it - only a real collapse that pitches the body back past upright does. Raise toward the setpoint to react earlier (risks engaging on a settle undershoot); lower (toward 0 / negative) for a later, safer catch */
#define SEESAW_FWD_CLAMP     0      /* PWM: max FORWARD common drive allowed during the anti-dive window. 0 = no forward drive at all while the board tips (strongest anti-dive; it rides the board down). Raise (e.g. 800/1500) if it instead falls BACKWARD off the board and needs some forward authority to stand. Reverse drive is never clamped */
#define SEESAW_DESCEND_SPEED (-3)  /* speed target while riding the tipped board down. NEGATIVE = down-slope BRAKE: with the position-integral dumped during descent, nothing was opposing gravity so it accelerated downhill ("下降速度太快"); a slight reverse target biases the P term to brake the roll. 0 = no brake (free roll); more negative (-5/-6) = stronger brake (too much may stall/reverse on a gentle slope) */
#define SEESAW_DESCEND_CYCLES 300  /* x10ms of creep after the tip (3s, enough to roll off the board), then speed -> 0 and it balances on the spot */

int  EXTI0_IRQHandler(void);
int  Balance(float angle, float gyro);
int  Velocity(int encoder_left, int encoder_right);
void Set_Pwm(int motor_left, int motor_right);
void Key(void);
int  PWM_Limit(int IN, int max, int min);
int  PWM_Ramp(int target, int current, int step);
u8   Turn_Off(float angle);
void Get_Angle(void);
void Set_Forward_Speed(int speed);   // command a forward-speed target (ramped internally)
int  myabs(int a);
float Control_GetBalanceTarget(void);
u8   Control_IsBalanceReady(void);
u8   Control_GetReadyCountdown(void);
u8   Control_GetMode(void);   // 0=BAL (pure balance) 1=TRACK (flat line-follow) 2=CLIMB (up-slope) 3=DOWN (riding tipped board down)
void Control_CalibrationTick(void);

extern volatile u32 g_isr_count;   // diagnostic: EXTI ISR entry counter (0 = never fires)

#endif
