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
 * dashes/falls BACKWARD -> more positive. Steps ~0.5 deg. */
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
 * moving; set back to 0 if dash/launch behaviour returns. */
#define VELOCITY_KI -0.8f

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
