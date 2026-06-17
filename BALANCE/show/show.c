#include "show.h"
#include "oled.h"
#include "ir_tracker.h"
#include "control.h"

/* Frame buffer owned by oled.c; blanked each refresh before drawing. */
extern u8 OLED_GRAM[128][8];

static void OLED_ShowSignedFixed16(u8 x, u8 y, float value)
{
	int ip, fp;
	if(value < 0)
	{
		OLED_ShowChar(x, y, '-', 16, 1);
		value = -value;
	}
	else
	{
		OLED_ShowChar(x, y, '+', 16, 1);
	}
	ip = (int)value;
	fp = (int)((value - ip) * 10);
	if(fp < 0) fp = 0;
	if(fp > 9) fp = 9;
	OLED_ShowNumber(x + 16, y, ip, 2, 16);
	OLED_ShowChar(x + 34, y, '.', 16, 1);
	OLED_ShowNumber(x + 44, y, fp, 1, 16);
}

/**************************************************************************
Function: Update the OLED display with current robot status:
          - Algorithm mode (DMP)
          - Left and right encoder counts
          - Balance angle and motor enable/stop state
Input   : none
Output  : none
**************************************************************************/
void oled_show(void)
{
	static u8 dashboard_entered = 0;
	static u8 resync_cnt = 0;

	/* Periodic controller resync (EMI self-heal): motor/EMI noise on the bit-banged
	 * SPI lines can corrupt the SSD1306 config registers and freeze/garble the
	 * display until power-cycle. Re-send the full command sequence every ~40 frames
	 * (~2.5s) so a glitched controller recovers on its own. The plain per-frame
	 * OLED_Display_On() only re-asserted charge-pump + display-on, not the addressing
	 * mode / remap / multiplex that also get corrupted - so this replaces it. */
	if(++resync_cnt >= 40) { OLED_Config(); resync_cnt = 0; }
	else                     OLED_Display_On();

	/* Blank the whole frame buffer each refresh so a previous big arrow
	   (or any stale pixels) never lingers under the normal screen. */
	{
		u8 px, pg;
		for(px = 0; px < 128; px++)
			for(pg = 0; pg < 8; pg++)
				OLED_GRAM[px][pg] = 0;
	}

	if(!Flag_Stop)
		dashboard_entered = 1;

	if(Flag_Stop && dashboard_entered == 0)
	{
		if(!Control_IsBalanceReady())
		{
			/* Kickstart the EXTI ISR, but ONLY while it isn't already running. The
			   MPU INT can be asserted (low) before EXTI_Init enables the falling-edge
			   IRQ, so no edge ever arrives and the ISR never fires; reading the FIFO
			   here releases/re-asserts INT and gets the edge train going. BUT once the
			   ISR IS firing it also calls Get_Angle() on the same bit-banged I2C bus -
			   doing a second Get_Angle() here concurrently corrupts the shared bus
			   (the ISR preempts mid-transaction). So only read from the main loop when
			   g_isr_count has NOT advanced since last frame (ISR dead/not-started);
			   otherwise let the ISR own the bus and just run the calibration tick on
			   the angle it already updated. */
			static u32 prev_isr = 0;
			u8 isr_alive = (g_isr_count != prev_isr);
			prev_isr = g_isr_count;
			if(!isr_alive) Get_Angle();
			Control_CalibrationTick();
		}

		OLED_ShowString(0, 0, "CAL");
		OLED_ShowString(30, 0, "N");                 /* ISR entry count: 0 = the interrupt never fires */
		OLED_ShowNumber(42, 0, g_isr_count, 5, 12);
		OLED_ShowString(96, 0, "I");
		OLED_ShowNumber(108, 0, INT, 1, 12);
		OLED_ShowSignedFixed16(30, 20, Angle_Balance);
		OLED_ShowString(0, 38, "T");
		OLED_ShowSignedFixed16(30, 38, Control_GetBalanceTarget());
		if(Control_IsBalanceReady())
		{
			OLED_ShowString(42, 48, "READY");
		}
		else
		{
			u8 remain = Control_GetReadyCountdown();
			OLED_ShowString(30, 46, "WAIT");
			OLED_ShowNumber(70, 46, remain, 2, 12);
			OLED_ShowString(92, 46, "s");
		}

		/* Live IR state so the sensors can be tested with motors OFF. The ISR
		   doesn't read IR while stopped, so refresh it here. 1 = black, 0 = white;
		   the three digits are LEFT / MIDDLE / RIGHT. */
		{
			u8 il, im, ir2;
			IR_Read(&il, &im, &ir2);
			OLED_ShowString(0, 52, "IR");
			OLED_ShowNumber(16, 52, il,  1, 12);
			OLED_ShowNumber(24, 52, im,  1, 12);
			OLED_ShowNumber(32, 52, ir2, 1, 12);
		}

		OLED_Refresh_Gram();
		return;
	}

	/* (Big turn-direction arrow removed: the dashboard below always shows now.) */

	/* Row 0: control mode + live IR sensor state (1 = sensor over the black line) */
	switch(Control_GetMode())
	{
		case 1:  OLED_ShowString(0, 0, "TRAK"); break;  // flat line-following
		case 2:  OLED_ShowString(0, 0, "CLMB"); break;  // climbing the up-slope
		case 3:  OLED_ShowString(0, 0, "DOWN"); break;  // riding the tipped board down
		default: OLED_ShowString(0, 0, "BAL "); break;  // pure balance
	}
	{
		u8 l, m, r;
		IR_Read(&l, &m, &r);
		OLED_ShowString(42, 0, "L");  OLED_ShowNumber(48, 0, l, 1, 12);
		OLED_ShowString(64, 0, "M");  OLED_ShowNumber(70, 0, m, 1, 12);
		OLED_ShowString(86, 0, "R");  OLED_ShowNumber(92, 0, r, 1, 12);
	}

	/* Row 20: left encoder count */
	OLED_ShowString(00, 20, "EncoLEFT");
	if(Encoder_Left < 0)
	{
		OLED_ShowString(80, 20, "-");
		OLED_ShowNumber(95, 20, -Encoder_Left, 3, 12);
	}
	else
	{
		OLED_ShowString(80, 20, "+");
		OLED_ShowNumber(95, 20,  Encoder_Left, 3, 12);
	}

	/* Row 30: right encoder count */
	OLED_ShowString(00, 30, "EncoRIGHT");
	if(Encoder_Right < 0)
	{
		OLED_ShowString(80, 30, "-");
		OLED_ShowNumber(95, 30, -Encoder_Right, 3, 12);
	}
	else
	{
		OLED_ShowString(80, 30, "+");
		OLED_ShowNumber(95, 30,  Encoder_Right, 3, 12);
	}

	/* Row 40: control-term breakdown. B = balance(upright PD), V = velocity(speed PI).
	   common_pwm = B + V (then clamped to +-6900 -> PL/PR). Whichever of B / V is
	   pinned at +-6900 regardless of angle is the runaway term:
	     B rails but doesn't change sign across upright -> balance polarity/setpoint
	     V rails -> speed loop wind-up (encoder feedback wrong / integral runaway) */
	{
		int b = g_balance_pwm;
		int v = g_velocity_pwm;
		OLED_ShowString(0, 40, "B");
		if(b < 0) { OLED_ShowString(8, 40, "-"); b = -b; }
		else        OLED_ShowString(8, 40, "+");
		OLED_ShowNumber(16, 40, b, 4, 12);
		OLED_ShowString(64, 40, "V");
		if(v < 0) { OLED_ShowString(72, 40, "-"); v = -v; }
		else        OLED_ShowString(72, 40, "+");
		OLED_ShowNumber(80, 40, v, 4, 12);
	}

	/* Row 50: tilt angle (1 decimal, for setpoint tuning) and motor enable state */
	{
		float a = Angle_Balance;
		int ip, fp;
		OLED_ShowString(0, 50, "Angle");
		if(a < 0) { OLED_ShowString(48, 50, "-"); a = -a; }
		else        OLED_ShowString(48, 50, "+");
		ip = (int)a;                  // integer part
		fp = (int)((a - ip) * 10);    // one decimal digit
		OLED_ShowNumber(53, 50, ip, 2, 12);
		OLED_ShowString(65, 50, ".");
		OLED_ShowNumber(71, 50, fp, 1, 12);
	}
	if( Flag_Stop) OLED_ShowString(90, 50, "OFF");
	if(!Flag_Stop) OLED_ShowString(90, 50, "ON ");

	/* Flush frame buffer to display */
	OLED_Refresh_Gram();
}
