#include "show.h"
#include "ir_tracker.h"
#include "control.h"

/* Frame buffer owned by oled.c; we blank it and draw a big turn arrow into it. */
extern u8 OLED_GRAM[128][8];

/* Fill a solid rectangle in the frame buffer (inclusive corners). */
static void OLED_FillRect(u8 xa, u8 ya, u8 xb, u8 yb)
{
	u8 x, y;
	for(x = xa; x <= xb; x++)
		for(y = ya; y <= yb; y++)
			OLED_DrawPoint(x, y, 1);
}

/**************************************************************************
Function: Draw a big full-screen arrow (head + tail bar) in screen space.
          dir: 0 = left (<-), 1 = right (->), 2 = up (^), 3 = down (v).
          The caller must have cleared the frame buffer first.
**************************************************************************/
static void OLED_BigArrow(u8 dir)
{
	int i, half;

	switch(dir)
	{
	case 1:                                       /* right -> */
		OLED_FillRect(8, 24, 76, 39);             /* tail bar on the left */
		for(i = 0; i <= 44; i++)
		{
			half = 30 * (44 - i) / 44;
			OLED_FillRect((u8)(75 + i), (u8)(32 - half), (u8)(75 + i), (u8)(32 + half));  /* tip on the right */
		}
		break;

	case 0:                                       /* left <- */
		OLED_FillRect(51, 24, 119, 39);           /* tail bar on the right */
		for(i = 0; i <= 44; i++)
		{
			half = 30 * (44 - i) / 44;
			OLED_FillRect((u8)(52 - i), (u8)(32 - half), (u8)(52 - i), (u8)(32 + half));  /* tip on the left */
		}
		break;

	case 2:                                       /* up ^ */
		OLED_FillRect(56, 30, 71, 60);            /* vertical tail bar (lower) */
		for(i = 6; i <= 34; i++)
		{
			half = 30 * (i - 6) / 28;
			OLED_FillRect((u8)(64 - half), (u8)i, (u8)(64 + half), (u8)i);  /* tip at top */
		}
		break;

	default:                                      /* down v */
		OLED_FillRect(56, 4, 71, 34);             /* vertical tail bar (upper) */
		for(i = 30; i <= 58; i++)
		{
			half = 30 * (58 - i) / 28;
			OLED_FillRect((u8)(64 - half), (u8)i, (u8)(64 + half), (u8)i);  /* tip at bottom */
		}
		break;
	}
}

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

	OLED_Display_On();

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
			/* This main-loop DMP read is REQUIRED: it is the angle source during
			   calibration AND it kickstarts the EXTI ISR. The MPU INT can already
			   be asserted (low) before EXTI_Init enables the falling-edge IRQ, so
			   no edge ever arrives and the ISR never fires; reading the FIFO here
			   releases/re-asserts INT and gets the edge train going. */
			Get_Angle();
			Control_CalibrationTick();
		}

		OLED_ShowString(0, 0, "CAL");
		OLED_ShowString(30, 0, "N");                 /* ISR entry count: 0 = the interrupt never fires */
		OLED_ShowNumber(42, 0, g_isr_count, 5, 12);
		OLED_ShowString(96, 0, "I");
		OLED_ShowNumber(108, 0, INT, 1, 12);
		OLED_ShowSignedFixed16(30, 20, Angle_Balance);
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
		OLED_Refresh_Gram();
		return;
	}

	/* While line-following, dedicate the whole screen to one big arrow:
	   -> / <- when steering, ^ ("go straight") otherwise. Nothing else shown.
	   err > 0 = line to the right (turn right); err < 0 = turn left; 0 = straight. */
	if(Flag_LineFollow)
	{
		static u8 shown_dir = 2;   /* arrow currently on screen (default: straight) */
		static u8 hold      = 0;   /* refresh cycles to wait before the next switch */
		u8 l, m, r, want;

		IR_Read(&l, &m, &r);                   /* live reading, 1 = sensor on the black line */
		if      (m && !l && !r) want = 2;      /* centred           -> straight */
		else if (r && !l)       want = 1;      /* line on the right -> turn right */
		else if (l && !r)       want = 0;      /* line on the left  -> turn left  */
		else                    want = 2;      /* lost / all-black / ambiguous -> straight (never stick) */

		if(hold) hold--;                       /* run down the lock-out window */
		if(want != shown_dir && hold == 0)     /* only switch once the >= 1 s lock expired */
		{
			shown_dir = want;
			hold = 20;                         /* hold this arrow >= ~1 s (20 x 50 ms) */
		}
		/* Only take over the screen for an actual turn. Going straight falls
		   through to the normal dashboard below.
		   Panel is mounted rotated, so rotate the arrow 90 deg CCW:
		   turn-right(->)->up, turn-left(<-)->down. */
		if(shown_dir != 2)
		{
			OLED_BigArrow(shown_dir == 1 ? 3 : 2);
			OLED_Refresh_Gram();
			return;
		}
	}

	/* Row 0: control mode + live IR sensor state (1 = sensor over the black line) */
	if(Flag_LineFollow) OLED_ShowString(0, 0, "LINE");
	else                OLED_ShowString(0, 0, "BAL ");
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
