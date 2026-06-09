#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

/* Mechanical equilibrium angle / setpoint (degrees).
 * This is the Pitch value at which the robot truly stands still (top-heavy => != 0).
 * Tune by binary search: if the robot drifts/falls toward the front, increase;
 * toward the back, decrease (steps of ~0.5 deg). 2.8 is the current bench value. */
#define Middle_angle 4.7f

/* Fine trim (degrees) added on top of the runtime-calibrated Balance_Target.
 * The jig may hold the bot slightly off the true balance point, leaving an
 * asymmetric "dashes/falls forward but recovers backward" behaviour.
 * Tune by binary search: dashes/falls FORWARD -> make MORE NEGATIVE;
 * dashes/falls BACKWARD -> more positive. Steps ~0.5 deg. */
#define BALANCE_TRIM (-1.4f)

/* Motor dead-zone compensation (PWM units, full scale = 7199).
 * The geared motors won't turn below ~600-1000 PWM, so small tilt errors
 * produce no motion until the robot leans far over. This offset is added to
 * every non-zero control output so the motor reacts to small tilts too.
 * Tune: raise if it still won't move at small angles; lower if it buzzes /
 * twitches / oscillates while standing near upright. Set to 0 to disable. */
#define MOTOR_DEADZONE 250

int  EXTI0_IRQHandler(void);
int  Balance(float angle, float gyro);
int  Velocity(int encoder_left, int encoder_right);
void Set_Pwm(int motor_left, int motor_right);
void Key(void);
int  PWM_Limit(int IN, int max, int min);
int  PWM_Ramp(int target, int current, int step);
int  PWM_Slew(int target, int current, int accel_step, int brake_step);
u8   Turn_Off(float angle);
void Get_Angle(void);
int  myabs(int a);
float Control_GetBalanceTarget(void);
u8   Control_IsBalanceReady(void);
u8   Control_GetReadyCountdown(void);
void Control_CalibrationTick(void);

extern volatile u32 g_isr_count;   // diagnostic: EXTI ISR entry counter (0 = never fires)

#endif
