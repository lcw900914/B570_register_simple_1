#include "control.h"
#include "ir_tracker.h"
#include "mpu6050.h"

/* ---- Debug switch (see DEBUG.md, step 5) ----
 * Uncomment to force the speed loop OFF (pure balance) for isolating the
 * "tilts to an angle then dashes forward and falls" fault.
 * Comment it back out for normal operation. */
/* #define DEBUG_BALANCE_ONLY */

/* Forward-speed target for the velocity loop. 0 = hold position (default).
 * Set by the line-follower each cycle (0 when Flag_LineFollow is off). */
static int Move_Target = 0;
static float Balance_Target = Middle_angle;  // Runtime-calibrated balance angle, captured while motors are OFF.
static u8 Balance_Ready = 0;
static u32 Cal_WindowStartISR = 0;  // g_isr_count at the calibration window start; 200Hz ISR tick = real-time timing
static float Gyro_Zero = 0.0f;  // stationary gyro offset captured during calibration; removed in Balance()
volatile u32 g_isr_count = 0;   // diagnostic: counts EXTI ISR entries; stays 0 if the interrupt never fires

/**************************************************************************
Function: Main control interrupt handler (triggered by MPU6050 INT pin,
          nominally every 5ms). Reads IMU and encoder data, runs PID loops,
          and updates motor PWM every 10ms.
Input   : none
Output  : none
**************************************************************************/
int EXTI0_IRQHandler(void)
{
	int Balance_Pwm, Velocity_Pwm;       // PID outputs
	int Motor_Left, Motor_Right;         // Final PWM values applied to motors
	int correction = 0, base_speed = 0;  // IR line-follow steering term and forward speed
	int ir_error = 0;                    // IR position error (this cycle)
	int early_brake = 0, encoder_sum = 0;
	int brake_target = 0;
	float angle_error = 0;
	static u8 Flag_Target;               // Toggles every 5ms to achieve 10ms control period
	static u8 line_speed_div = 0;
	static u8 brake_active = 0;
	static int brake_pwm = 0;

	g_isr_count++;                       // diagnostic: every ISR entry, BEFORE the INT==0 guard

	if(INT == 0)
	{
		EXTI->PR = 1<<0;                 // Clear external interrupt pending flag (EXTI line 0 = PA0)

		Flag_Target = !Flag_Target;
		Get_Angle();                     // Read IMU and compute tilt angle
		Encoder_Left  =  Read_Encoder(3); // left NOT negated: forward now reads + on BOTH wheels (was -, which cancelled the right wheel in the velocity loop's L+R sum)
		Encoder_Right = -Read_Encoder(4); // right encoder

		if(Flag_Target == 1)             // Execute control loop every 10ms (skip one 5ms cycle)
			return 0;

		/* Key() moved to the main loop (MiniBalance.c): this ISR is unreliable,
		   so the button is now polled in the always-running main loop. */

		Balance_Pwm  = Balance(Angle_Balance, Gyro_Balance);     // Upright PD control

		/* --- IR line following (differential steering, gated by Flag_LineFollow) --- */
		if(Flag_LineFollow)
		{
			correction = IR_GetCorrection();         // PD steering term (reads the 3 sensors)
			correction = PWM_Limit(correction, 350, -350);  // keep steering from overpowering balance
			ir_error   = IR_GetLastError();          // error just computed (no re-read)
			line_speed_div++;
			if(line_speed_div >= 3) line_speed_div = 0;
			base_speed = (line_speed_div == 0) ? LINE_BASE_SPEED : 0;  // average speed = LINE_BASE_SPEED / 3
			if(myabs(ir_error) >= 2)                 // sharp turn -> slow down (x0.75)
				base_speed = base_speed * 3 / 4;
		}
		else
		{
			correction = 0;
			base_speed = 0;
		}
		Move_Target = base_speed;                    // forward target read by Velocity()

		Velocity_Pwm = Velocity(Encoder_Left, Encoder_Right);    // Speed PI control

		/* Gentle speed brake only. Hard threshold rescue made the body fall
		 * immediately on this hardware, so keep the balance loop continuous.
		 */
		angle_error = Angle_Balance - (Balance_Target + BALANCE_TRIM);
		if(angle_error < 0) angle_error = -angle_error;
		encoder_sum = Encoder_Left + Encoder_Right;
		if(angle_error > 1.4f)
			brake_active = 1;
		else if(angle_error < 0.7f)
			brake_active = 0;

#ifdef DEBUG_BALANCE_ONLY
		brake_target = 0;
#else
		if(brake_active && myabs(encoder_sum) > 3)
			brake_target = PWM_Limit(-encoder_sum * 3, 250, -250);
		else
			brake_target = 0;
#endif

		brake_pwm = (brake_pwm * 7 + brake_target) / 8;
		early_brake = brake_pwm;

		Motor_Left  = Balance_Pwm + Velocity_Pwm + early_brake - correction;
		Motor_Right = Balance_Pwm + Velocity_Pwm + early_brake + correction;

		Motor_Left  = PWM_Limit(Motor_Left,   6900, -6900);      // Clamp to valid PWM range
		Motor_Right = PWM_Limit(Motor_Right,  6900, -6900);

		if(Turn_Off(Angle_Balance) == 0)                         // Only drive if tilt is safe
			Set_Pwm(Motor_Left, Motor_Right);
	}
	return 0;
}

/**************************************************************************
Function: Balance PD control - computes PWM to keep robot upright
Input   : Angle - current tilt angle (degrees)
          Gyro  - current angular velocity (degrees/s)
Output  : Balance PWM value
**************************************************************************/
int Balance(float Angle, float Gyro)
{
	float Balance_Kp = 560, Balance_Kd = 12;    // softer damping to reduce fore-aft body shake
	float Angle_bias, Gyro_bias;
	float abs_bias;
	float limit_f;
	int   balance_limit;
	int   balance;

	Angle_bias = (Balance_Target + BALANCE_TRIM) - Angle;   // calibrated setpoint + manual trim, minus current angle
	Gyro_bias  = 0 - (Gyro - Gyro_Zero);          // remove the stationary gyro offset captured at calibration
	balance    = Balance_Kp * Angle_bias + Gyro_bias * Balance_Kd;   // PD output (direction flipped: wheels now drive toward the lean)

	/* Nonlinear catch with a smooth limit:
	 * small errors stay gentle; larger errors get more authority without a hard
	 * step in PWM that can show up as a jerk.
	 */
	abs_bias = Angle_bias;
	if(abs_bias < 0) abs_bias = -abs_bias;
	limit_f = 1100.0f + abs_bias * 350.0f;
	if(limit_f > 2800.0f) limit_f = 2800.0f;   // lower max catch force to prevent growing oscillation
	balance_limit = (int)limit_f;
	balance = PWM_Limit(balance, balance_limit, -balance_limit);
	return balance;
}

/**************************************************************************
Function: Velocity PI control - computes PWM correction to maintain
          zero average wheel speed (robot stays on spot)
Input   : encoder_left  - left wheel encoder count (this 10ms period)
          encoder_right - right wheel encoder count (this 10ms period)
Output  : Velocity control PWM value??
**************************************************************************/
int Velocity(int encoder_left, int encoder_right)
{
#ifdef DEBUG_BALANCE_ONLY
	float Velocity_Kp = 0,   Velocity_Ki = 0;     // DEBUG: speed loop forced OFF -> pure balance (isolation test)
#else
	float Velocity_Kp = -45, Velocity_Ki = -0.01; // tiny pullback; reduce oscillation while keeping some recovery
#endif
	static float velocity, Encoder_Least, Encoder_bias;
	static float Encoder_Integral;

	/* Speed error = target - actual speed. Move_Target is 0 for stationary
	 * balance, or the line-follow forward speed when tracking. */
	Encoder_Least   = Move_Target - (encoder_left + encoder_right);

	/* First-order low-pass filter: 80% previous + 20% new */
	Encoder_bias   *= 0.8;
	Encoder_bias   += Encoder_Least * 0.2;

	/* Integrate filtered error (10ms period) */
	Encoder_Integral += Encoder_bias;
	if(Encoder_Integral >  1000) Encoder_Integral =  1000;   // Anti-windup: keep speed loop from building a launch command
	if(Encoder_Integral < -1000) Encoder_Integral = -1000;

	velocity = -Encoder_bias * Velocity_Kp - Encoder_Integral * Velocity_Ki;

	/* Reset integrator when motors are disabled or stopped */
	if(Turn_Off(Angle_Balance) == 1 || Flag_Stop == 1)
		Encoder_Integral = 0;

	return velocity;
}

/**************************************************************************
Function: Write PWM values and motor direction to the H-bridge driver
Input   : motor_left  - PWM for left motor  (positive=forward, negative=backward)
          motor_right - PWM for right motor
Output  : none
**************************************************************************/
void Set_Pwm(int motor_left, int motor_right)
{
	int pwm_left, pwm_right;

	/* Left motor (A): set direction, then magnitude + dead-zone offset */
	if(motor_left > 0)      { AIN1 = 1; AIN2 = 0; pwm_left =  motor_left + MOTOR_DEADZONE; }
	else if(motor_left < 0) { AIN1 = 0; AIN2 = 1; pwm_left = -motor_left + MOTOR_DEADZONE; }
	else                    { AIN1 = 0; AIN2 = 0; pwm_left = 0; }  // exactly balanced: no drive
	if(pwm_left > 7199) pwm_left = 7199;     // clamp to PWM full scale (ARR)
	PWMA = pwm_left;

	/* Right motor (B) */
	if(motor_right > 0)      { BIN1 = 1; BIN2 = 0; pwm_right =  motor_right + MOTOR_DEADZONE; }
	else if(motor_right < 0) { BIN1 = 0; BIN2 = 1; pwm_right = -motor_right + MOTOR_DEADZONE; }
	else                     { BIN1 = 0; BIN2 = 0; pwm_right = 0; }
	if(pwm_right > 7199) pwm_right = 7199;
	PWMB = pwm_right;
}

/**************************************************************************
Function: Clamp a PWM value to the specified range
Input   : IN  - input value
          max - upper limit
          min - lower limit
Output  : clamped output value
**************************************************************************/
int PWM_Limit(int IN, int max, int min)
{
	int OUT = IN;
	if(OUT > max) OUT = max;
	if(OUT < min) OUT = min;
	return OUT;
}

int PWM_Ramp(int target, int current, int step)
{
	if(target > current + step) return current + step;
	if(target < current - step) return current - step;
	return target;
}

int PWM_Slew(int target, int current, int accel_step, int brake_step)
{
	int step;

	if((target > 0 && current < 0) || (target < 0 && current > 0))
		step = brake_step;
	else if(myabs(target) < myabs(current))
		step = brake_step;
	else
		step = accel_step;

	return PWM_Ramp(target, current, step);
}

/**************************************************************************
Function: Toggle the robot stop state when the key is pressed
Input   : none
Output  : none
**************************************************************************/
void Key(void)
{
	static u8  click_pending = 0;   // waiting to see if a 2nd click arrives
	static u16 dbl_timer     = 0;   // double-click window countdown (10ms ticks)
	u8 tmp = click();

	if(tmp == 1)
	{
		if(click_pending)               // 2nd click within window -> DOUBLE
		{
			Flag_LineFollow = !Flag_LineFollow;  // toggle line-follow / pure balance
			click_pending = 0;
		}
		else                            // 1st click -> open the window
		{
			click_pending = 1;
			dbl_timer = 12;             // ~600ms at the 50ms main-loop rate; easier double-click window
		}
		return;
	}

	if(click_pending)                   // window running, no 2nd click yet
	{
		if(dbl_timer > 0) dbl_timer--;
		if(dbl_timer == 0)              // window closed -> it was a SINGLE click
		{
			if(Flag_Stop == 0 || Balance_Ready)
				Flag_Stop = !Flag_Stop; // only allow ON after calibration is ready
			click_pending = 0;
		}
	}
}

/**************************************************************************
Function: Check for fault conditions and disable motors if necessary
          (tilt > 40 degrees or stop flag set)
Input   : angle - current tilt angle
Output  : 1 if motors should be off, 0 if normal operation
**************************************************************************/
u8 Turn_Off(float angle)
{
	u8 temp;
	if(angle < -40 || angle > 40 || Flag_Stop == 1)
	{
		temp = 1;
		AIN1 = 0;
		AIN2 = 0;
		BIN1 = 0;
		BIN2 = 0;
	}
	else
		temp = 0;
	return temp;
}

/**************************************************************************
Function: Read attitude from DMP and update balance control variables
Input   : none
Output  : none (updates Angle_Balance and Gyro_Balance globals)
**************************************************************************/
void Get_Angle(void)
{
	Read_DMP();                    // Read quaternion from DMP FIFO
	MPU_ClearINT();                // release the latched MPU INT so PA12 makes a fresh falling edge -> keeps the EXTI ISR firing
	Angle_Balance = Pitch;         // Forward/backward tilt
	Gyro_Balance  = -gyro[0];      // hardware-confirmed damping axis; gyro[1] caused immediate fall
}

void Control_CalibrationTick(void)
{
	static long  sum_angle10 = 0;
	static float filt = 0.0f;
	static u8    filt_init = 0;
	static int   ref10 = 0;        // filtered angle captured at the window start (fixed anchor)
	static float sum_gyro = 0.0f;  // accumulates Gyro_Balance over the stable window -> offset
	static u32   nsamp = 0;        // samples accumulated in the current window (for averaging)
	int angle10;

	if(Flag_Stop == 0 || Balance_Ready || Angle_Balance <= -45.0f || Angle_Balance >= 45.0f)
		return;   /* widened from +-15: the rest angle can read large if the MPU zero is tilted, and that was blocking calibration entirely */

	/* Low-pass the RAW angle first. The DMP has ~+-0.2 deg of electronic noise,
	 * so two raw samples can differ by 0.4 deg even when the jig is dead still -
	 * a 0.3 deg test on the raw value can NEVER pass and keeps resetting the
	 * window (countdown frozen). Filtering removes the jitter, leaving the true
	 * steady angle; the 0.3 deg test then runs on the clean value. */
	if(!filt_init) { filt = Angle_Balance; filt_init = 1; }
	filt = filt * 0.85f + Angle_Balance * 0.15f;

	if(filt >= 0)
		angle10 = (int)(filt * 10.0f + 0.5f);
	else
		angle10 = (int)(filt * 10.0f - 0.5f);

	/* Start a fresh window, or restart it if the filtered angle drifted >0.3 deg
	 * from the anchor. The window DURATION is measured in g_isr_count (steady
	 * 200 Hz from the ISR), NOT in main-loop samples, so READY always means a
	 * real ~15 s of stillness regardless of how fast the main loop spins. */
	if(nsamp == 0 || myabs(angle10 - ref10) > 3)  /* fresh start, or moved -> restart window */
	{
		Cal_WindowStartISR = g_isr_count;
		ref10 = angle10;
		sum_angle10 = 0;
		sum_gyro = 0.0f;
		nsamp = 0;
	}

	sum_angle10 += angle10;
	sum_gyro    += Gyro_Balance;
	nsamp++;

	if((u32)(g_isr_count - Cal_WindowStartISR) >= 3000)   /* 3000 ISR ticks @200Hz = real 15 s steady */
	{
		Balance_Target = (sum_angle10 / (long)nsamp) / 10.0f;
		Gyro_Zero      = sum_gyro / (float)nsamp;          /* remove in Balance() so a still bot stops driving */
		Balance_Ready  = 1;
	}
}

/**************************************************************************
Function: Integer absolute value
Input   : a - signed integer
Output  : |a|
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a < 0) temp = -a;
	else      temp =  a;
	return temp;
}

float Control_GetBalanceTarget(void)
{
	return Balance_Target;
}

u8 Control_IsBalanceReady(void)
{
	return Balance_Ready;
}

u8 Control_GetReadyCountdown(void)
{
	u32 elapsed;

	if(Balance_Ready) return 0;
	elapsed = (u32)(g_isr_count - Cal_WindowStartISR);   // ISR ticks at a steady 200 Hz
	if(elapsed >= 3000) return 0;
	return (u8)(15 - elapsed / 200);                     // whole seconds of stillness remaining
}
