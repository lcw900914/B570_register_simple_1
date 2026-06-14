#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

/* Mechanical equilibrium angle / setpoint (degrees).
 * This is the Pitch value at which the robot truly stands still (top-heavy => != 0).
 * Tune by binary search: if the robot drifts/falls toward the front, increase;
 * toward the back, decrease (steps of ~0.5 deg). 2.8 is the current bench value. */
#define Middle_angle 7.1f

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

/* Speed-loop integral gain. Was hard-coded 0 to avoid the "winds up then
 * launches" fault. Re-enable with a SMALL value to hold cruising speed while
 * moving; set back to 0 if dash/launch behaviour returns.
 * -1.5: more push per unit of integral -> uphill force builds FASTER (at -0.8
 * the climb push took several seconds to wind up; the bot stalled mid-ramp and
 * only made it when entry momentum carried it). If flat ground shows slow
 * forward-backward hunting or launch, back off to -1.2 / -1.0. */
#define VELOCITY_KI -1.5f

/* ===== Seesaw (蹺蹺板) tip-over handling ================================ */
/* Absolute-angle tip detection (thresholds are raw Angle_Balance = the OLED Angle
 * value, NOT relative to the balance setpoint). While line-following:
 *   ARM  : Angle_Balance climbs ABOVE SEESAW_CLIMB_PITCH (steep forward lean while
 *          driving up the slope) and holds it for SEESAW_CLIMB_CONFIRM cycles.
 *   TRIP : once armed, Angle_Balance drops BELOW SEESAW_TIP_PITCH (pitched right
 *          back to the downhill side) -> the board has tipped -> line-follow OFF,
 *          dump the climb push, ride the new downhill in pure balance.
 * A steady climb sits up near +SEESAW_CLIMB_PITCH and never reaches the deeply
 * NEGATIVE trip angle, so it cannot false-fire mid-climb (the problem the
 * relative-lean / gyro versions had). */
#define SEESAW_CLIMB_PITCH   17.0f  /* deg of forward pitch (Angle_Balance) that arms the detector. Watch the OLED Angle while climbing - it must actually reach this, or lower it */
#define SEESAW_CLIMB_CONFIRM 30     /* x10ms the steep climb pitch must persist (0.3s) - filters accel/bump transients */
#define SEESAW_TIP_PITCH    (0.0f) /* deg: once armed, Angle_Balance dropping below this fires the full tip (switch to pure-balance descent). Set to 0 = switch the moment the body pitches back through level, catching the forward dive early. Lower (negative) to fire later; must stay BELOW SEESAW_BRAKE_PITCH so the anti-dive brake engages first */
/* ---- Early anti-dive (acts BEFORE the full -15 trip) ----
 * The full trip at SEESAW_TIP_PITCH is deliberately late - by the time the body
 * has pitched all the way back to -15, the wheels have already slammed forward
 * (that slam is what drives the angle so negative). So the moment an armed climb
 * starts collapsing past SEESAW_BRAKE_PITCH we pre-empt the dive: dump the climb
 * integral and clamp forward drive. */
#define SEESAW_BRAKE_PITCH   3.0f   /* deg: once armed, Angle_Balance falling BELOW this starts the anti-dive (dump push + clamp forward). MUST sit below the upright setpoint (~7) so plain standing / a hump crest that settles upright does NOT engage it - only a real collapse that pitches the body back past upright does. Raise toward the setpoint to react earlier (risks engaging on a settle undershoot); lower (toward 0 / negative) for a later, safer catch */
#define SEESAW_FWD_CLAMP     0      /* PWM: max FORWARD common drive allowed during the anti-dive window. 0 = no forward drive at all while the board tips (strongest anti-dive; it rides the board down). Raise (e.g. 800/1500) if it instead falls BACKWARD off the board and needs some forward authority to stand. Reverse drive is never clamped */
#define SEESAW_DESCEND_SPEED 0     /* forward creep target while riding the tipped board down. 0 = DON'T push forward: gravity already rolls it down the slope, and the climb integral was just dumped, so adding forward speed only feeds the dive. Set slightly negative for a gentle down-slope brake, or small positive only if it actually parks on the board */
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
void Control_CalibrationTick(void);

extern volatile u32 g_isr_count;   // diagnostic: EXTI ISR entry counter (0 = never fires)

#endif
