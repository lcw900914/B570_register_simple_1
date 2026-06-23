#include "control.h"
#include "ir_tracker.h"
#include "mpu6050.h"

/* ---- Debug switch (see DEBUG.md, step 5) ----
 * Uncomment to force the speed loop OFF (pure balance) for isolating the
 * "tilts to an angle then dashes forward and falls" fault.
 * Comment it back out for normal operation. */
//#define DEBUG_BALANCE_ONLY    /* normal operation: speed-brake ENABLED (brakes the forward translation when the body leans, countering the "leans forward -> surges to balance" rush) */

/* Forward-speed target for the velocity loop. 0 = hold position (default).
 * Set by the line-follower each cycle (0 when Flag_LineFollow is off). */
static int Move_Target = 0;          // ramped forward-speed target actually used by Velocity()
static int Move_Target_Cmd = 0;      // commanded forward speed (line-follower or Set_Forward_Speed()); Move_Target ramps toward this
static float Forward_Lean = 0.0f;    // setpoint feed-forward tilt while moving (see FORWARD_LEAN_GAIN)
static float Balance_Target = Middle_angle;  // Runtime-calibrated balance angle, captured while motors are OFF.
static u8 Balance_Ready = 1;
static u32 Cal_WindowStartISR = 0;  // g_isr_count at the calibration window start; 200Hz ISR tick = real-time timing
static float Gyro_Zero = 0.0f;  // stationary gyro offset captured during calibration; removed in Balance()
static u8 Velocity_Reset_Req = 0;  // one-shot: Velocity() dumps its integral+filter. Set when the seesaw tips - the wound-up CLIMB push (up to Ki*clamp PWM) would otherwise launch the bot down the new downhill
static u8 Seesaw_Armed = 0;        // 1 = a sustained steep forward climb was confirmed; latched until tip/disarm. Gates the climb slowdown so slowing never suppresses the (full-speed) arming
static u8 Seesaw_Descending = 0;   // 1 = riding the tipped board down (post-tip descent window). Tells Velocity() to drop its position-hold integral so it doesn't drive the bot BACKWARD against the downhill roll
static u8 Track_Mode = 0;          // 1 = FLAT line-following (not climbing/descending/balancing). Selects the line-follow setpoint (MIDDLE_ANGLE_TRACK) and speed gain (VELOCITY_KP_TRACK); 0 = slope/balance values
volatile u32 g_isr_count = 0;   // diagnostic: counts EXTI ISR entries; stays 0 if the interrupt never fires
volatile int g_pwm_left  = 0;   // diagnostic: last SIGNED motor command sent to Set_Pwm (left);  +=forward, -=reverse
volatile int g_pwm_right = 0;   // diagnostic: last SIGNED motor command sent to Set_Pwm (right)
volatile int g_balance_pwm  = 0;  // diagnostic: balance (upright PD) term this cycle
volatile int g_velocity_pwm = 0;  // diagnostic: velocity (speed PI) term this cycle -> if THIS rails at +-6900 the speed loop is the runaway

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
	int early_brake = 0;
	int brake_target = 0;
	int tip_brake = 0;                   // this-cycle anti-dive flag: set while an armed climb is collapsing (board tipping), clamps the forward motor drive that would otherwise launch the bot
	static u8 Flag_Target;               // Toggles every 5ms to achieve 10ms control period
	static int brake_pwm = 0;
	static u8 center_count = 0;          // consecutive centred (010) cycles, counted ONLY after a lost (000) state; debounces the 000->010 re-acquire so it doesn't lurch to full speed
	static u8 was_lost = 0;              // 1 = the line was just lost (000); cleared by any turn. Gates the 000->010 speed-up delay
	static u8 was_hard = 0;              // 1 = just came out of a hard turn (+-2); eases the turn-exit speed-up so it doesn't fling off the line after pivoting
	static u16 lost_count = 0;           // cycles continuously lost (000); drives the lost-recovery escalation (arc -> spin -> stop)
	static int straight_corr = 0;        // small fading steering residual applied on the straight (010) to smooth the sawtooth
	static u8 in_pivot = 0;             // 1 = hard tank pivot active
	static u8 pivot_min_count = 0;      // ignore old-line 010 briefly after pivot starts
	static u8 pivot_seen_lost = 0;      // pivot can finish only after seeing 000, then 010
	static int pivot_dir = 0;           // latched hard-turn direction
	static int last_pivot_dir = 0;      // direction of the most recent committed pivot; used as the search hint when the line vanishes straight to 000 (no last side known)
	static int pivot_pause = 0;         // cycles left in the rebalance PAUSE between small pivot steps (stepped big-turn: small turn -> stabilise -> small turn ...)
	static u8  overrot_done = 0;        // 1 = already added the over-rotation past centre for this pivot (one-shot, so the heading finishes ~90deg instead of exiting under-rotated)
	static u8 lost_from_pivot = 0;      // 1 = the 000 began while a pivot was rotating (between old/new line - keep rotating); 0 = blew straight past the corner (line is BEHIND the axle - must BACK UP, no rotation can reach it)
	static u8 reached_center = 0;       // 1 = have hit centre at least once since the turn started -> later +-1 drifts are gentle fine-align, not a fresh hard turn
	static int post_turn_back = 0;      // cycles left in the "reverse ~1s after a turn" phase (0 = inactive)
	static u8  did_hard_turn = 0;       // 1 = a real turn (pivot or +-2) happened; triggers the post-turn backup when the bot re-centres, then clears

	g_isr_count++;                       // diagnostic: every ISR entry, BEFORE the INT==0 guard

	/* ALWAYS acknowledge the EXTI pending bit FIRST, even if INT has already gone
	 * back high (the MPU INT is any-read-clear, so a fast release or a glitch can
	 * leave the pin high by the time we run). The old code cleared PR only INSIDE
	 * the INT==0 branch: an edge that had already released left PR latched, so the
	 * ISR immediately re-fired forever (interrupt storm). The CPU was pinned in
	 * this handler, the main loop and OLED froze, and the motors kept driving on
	 * their LAST PWM with no fresh angle - the "當機就亂衝、沒在看角度" failure.
	 * Clearing unconditionally costs at most one skipped sample; the next real
	 * falling edge starts a clean cycle. */
	EXTI->PR = 1<<0;                     // Clear external interrupt pending flag (EXTI line 0 = PA0)

	if(INT == 0)
	{
		Flag_Target = !Flag_Target;
		Get_Angle();                     // Read IMU and compute tilt angle
		/* Encoder channels were SWAPPED in code vs the actual wiring (hand-spin test):
		 * TIM3 is physically the RIGHT wheel, TIM4 the LEFT. With the old labels the
		 * negation landed on the wrong wheel and the velocity loop's L+R sum read ~0
		 * while driving straight -> NO speed feedback, speed brake and exit brake
		 * never fired -> the chronic forward surge. */
		Encoder_Right = Read_Encoder(3);  // RIGHT wheel (verified: rolled forward by hand -> counts +)
		Encoder_Left  = Read_Encoder(4);  // LEFT wheel (verified after the wiring fix: raw counts were already + for forward, so NO negation - with the negation it read - on a hand-forward roll)

		if(Flag_Target == 1)             // Execute control loop every 10ms (skip one 5ms cycle)
			return 0;

		/* Key() moved to the main loop (MiniBalance.c): this ISR is unreliable,
		   so the button is now polled in the always-running main loop. */

		/* ---- Seesaw (蹺蹺板) tip-over detector (see SEESAW_* in control.h) ----
		 * Climbing holds a sustained lean toward the hill; the moment the board
		 * tips, the slope reverses and the equilibrium lean flips to the OTHER
		 * side of the setpoint. That one-side-then-swing-past signature never
		 * occurs in normal flat driving. On trigger: drop line-follow, dump the
		 * wound-up climb integral, and ride the new downhill in pure balance
		 * with a gentle forward creep. */
		{
			static u16 climb_cnt   = 0;   // consecutive cycles the steep climb pitch has held (while not armed)
			static u16 descend_cnt = 0;   // remaining creep cycles after the tip
			static u16 settle_cnt  = 0;   // consecutive settled-upright cycles while armed -> self-disarm a false arm (e.g. a plain hump crest, not a seesaw)
			static float climb_peak = 0;  // highest Angle_Balance seen since arming; the tip fires when the angle drops SEESAW_TIP_DROP below it
			/* arm latch is the file-scope Seesaw_Armed (so the climb slowdown can read it) */

			/* Tip detection by a SHARP DROP from the climb peak (raw Angle_Balance = OLED Angle):
			 *   ARM  : Angle_Balance climbs PAST SEESAW_CLIMB_PITCH (~10; the climb peaks
			 *          ~17-20) and holds it for SEESAW_CLIMB_CONFIRM cycles. climb_peak is
			 *          then tracked as the high-water mark.
			 *   TRIP : once armed, the pitch FALLS more than SEESAW_TIP_DROP below climb_peak
			 *          (e.g. ~20 -> -5, the board flipping under it), OR rises past
			 *          SEESAW_TIP_FWD_PITCH (backup, forward dive).
			 *   ANTI-DIVE: once armed, if the pitch is below SEESAW_BRAKE_PITCH but the drop
			 *          hasn't hit the threshold yet, dump the climb push and clamp forward
			 *          drive early, before the wheels slam.
			 * The drop is relative to the actual climb height, so it adapts to the slope;
			 * normal climb wobble is far smaller than SEESAW_TIP_DROP so it can't false-trip,
			 * and settle_cnt disarms if an armed climb just levels off instead of tipping. */
			if(Flag_LineFollow && Flag_Stop == 0)
			{
				descend_cnt = 0;
				Seesaw_Descending = 0;
				if(!Seesaw_Armed)                // not armed: wait for a sustained steep forward climb
				{
					if(Angle_Balance > SEESAW_CLIMB_PITCH) { if(++climb_cnt >= SEESAW_CLIMB_CONFIRM) { Seesaw_Armed = 1; settle_cnt = 0; climb_peak = Angle_Balance; } }
					else                                     climb_cnt = 0;
				}
				else
				{
					if(Angle_Balance > climb_peak) climb_peak = Angle_Balance;   // track the climb high-water mark

					if((climb_peak - Angle_Balance > SEESAW_TIP_DROP && Angle_Balance < SEESAW_TIP_FALL_TO)  // TIPPED: a sharp DROP from the climb peak AND the angle actually fell to a tipped-low value (the AND rejects a flexing-board spike that inflates the peak then returns to a still-climbing angle)
					   || Angle_Balance > SEESAW_TIP_FWD_PITCH)                                          // OR dove forward past the climb band
					{
						Flag_LineFollow    = 0;                 // drop to pure balance (the else-branch below also clears all pivot state this same cycle)
						Velocity_Reset_Req = 1;                 // dump the climb push NOW - on the new downhill it would launch the bot
						Set_Forward_Speed(SEESAW_DESCEND_SPEED);
						descend_cnt = SEESAW_DESCEND_CYCLES;
						Seesaw_Descending = 1;                  // ride-down window: Velocity() drops its position-hold integral so it won't reverse against the roll
						Seesaw_Armed = 0;
						climb_cnt   = 0;
					}
					else if(Angle_Balance < SEESAW_BRAKE_PITCH) // collapsing past upright but the drop hasn't hit the threshold yet: kill the launch energy EARLY (dump push + clamp forward) before the wheels slam
					{
						Velocity_Reset_Req = 1;                 // dump the wound-up climb integral every cycle while collapsing -> no sustained forward push left to launch on
						tip_brake = 1;                          // clamp the forward balance drive this cycle (see SEESAW_FWD_CLAMP at the motor mix)
						settle_cnt = 0;                         // it's moving down, not settled
					}
					else                                        // armed, no big drop yet: still climbing or leveling off without tipping
					{
						float d = Angle_Balance - Balance_Target;
						if(d < 0) d = -d;
						if(d < 2.0f) { if(++settle_cnt >= 100) { Seesaw_Armed = 0; climb_cnt = 0; settle_cnt = 0; } }  // settled upright ~1s with no tip -> that was a false arm (hump, not seesaw), disarm
						else settle_cnt = 0;
					}
				}
			}
			else
			{
				Seesaw_Armed = 0;
				climb_cnt  = 0;
				settle_cnt = 0;
				climb_peak = 0;
				if(descend_cnt > 0)               // post-tip descent: creep forward, then stop
				{
					descend_cnt--;
					if(descend_cnt == 0 || Flag_Stop) { Set_Forward_Speed(0); descend_cnt = 0; Seesaw_Descending = 0; }
				}
			}
		}

		/* --- IR line following (differential steering, gated by Flag_LineFollow) ---
		 * Also gated by Flag_Stop so handling the bot while stopped can't latch a
		 * pivot that then fires the moment it starts. */
		if(Flag_LineFollow && Flag_Stop == 0)
		{
			correction = IR_GetCorrection();         // PD steering term (reads the 3 sensors)
			ir_error   = IR_GetLastError();          // error just computed (no re-read)

			if(in_pivot || myabs(ir_error) >= 2)     // a REAL turn is happening (pivot, or line fully off to one side)
				did_hard_turn = 1;                   // ...remembered so the post-turn backup fires when we re-centre

			/* Pivot lifetime watchdog: a hard pivot ticks down EVERY cycle (not only
			 * while centred). When the minimum has elapsed, release the latch so
			 * normal handling resumes - it exits at the next 010, or re-enters a
			 * fresh pivot if the corner is still hard. Bounds every pivot so a turn
			 * that never goes all-white (000) can't spin forever. */
			if(in_pivot)
			{
				if(pivot_min_count > 0) pivot_min_count--;
				if(pivot_min_count == 0)
				{
					in_pivot = 0;
					pivot_seen_lost = 0;
					pivot_dir = 0;
					pivot_pause = PIVOT_PAUSE_CYCLES;   // small turn done -> pause & rebalance before the next step
				}
			}
			else if(pivot_pause > 0) pivot_pause--;     // counting down the between-steps rebalance pause

			/* Corners are handled by the ALL-WHITE (000) path now: +-2 no longer
			 * escalates to a pivot by itself. On this sensor bar (2cm lookahead) a
			 * real 90deg corner reads 000 almost immediately anyway, and +-2-triggered
			 * pivots kept false-firing on small curves' over-corrections. Just
			 * remember which side the line was last fully off-centre - the 000
			 * pivot uses it as its turn direction. */
			if(myabs(ir_error) >= 2)
				last_pivot_dir = (ir_error > 0) ? 1 : -1;

			if(!IR_LineLost()) lost_from_pivot = 0;      // any sighting of the line ends the current lost episode

			if(IR_LineLost() && in_pivot && pivot_dir != 0 && lost_count < LOST_GIVEUP_CYCLES)  // pivoting with the line gone (old line cleared, or lost-search): keep rotating to find the new 010
			{
				lost_from_pivot = 1;                  // this 000 happened mid-rotation: between the old and new line, rotation is the right recovery (do NOT back up)
				pivot_seen_lost = 1;
				base_speed = 0;                       // motor mixing spins via PIVOT_OUTER/PIVOT_INNER toward pivot_dir
				center_count = 0;
				was_lost = 0;
				was_hard = 1;
				if(lost_count < 1000) lost_count++;   // KEEP counting while lost-pivoting so a truly lost bot still reaches give-up (was reset to 0 here, which could search forever)
				straight_corr = 0;
			}
			else if(IR_LineLost())                   // all-white (000), NOT mid-pivot: line truly lost
			{
				/* TEMPORARY BALANCE MODE: with no line, just balance on the spot - do
				 * NOT drive forward / reverse / search (that was the "no line -> charge
				 * forward" behaviour). Resume line-following only when black is seen. */
				base_speed      = 0;
				correction      = 0;
				in_pivot        = 0;
				pivot_min_count = 0;
				pivot_seen_lost = 0;
				pivot_dir       = 0;
				was_lost        = 1;   // re-acquire via the upright gate when the line returns
				was_hard        = 0;
				center_count    = 0;
				straight_corr   = 0;
				reached_center  = 0;
			}
			else if(in_pivot && ir_error != 0)       // the 000-pivot caught the line OFF-centre: keep rotating (same latched direction) until it reaches the centre sensor
			{
				base_speed = 0;                      // motor mixing keeps spinning via PIVOT_OUTER/PIVOT_INNER toward pivot_dir
				center_count = 0;
				was_lost = 0;
				was_hard = 1;                        // ease the speed-up when we finally re-centre (anti-fling-off)
				lost_count = 0;
				straight_corr = 0;
			}
			else if(myabs(ir_error) == 1)            // slight drift (011/110): GENTLE moving ARC, NOT an in-place pivot.
			{
				/* Pivoting on every small drift was the big curve oscillation: the bot
				 * stop-spun toward the line, overshot, then spun back (一頓一頓的折線).
				 * Here it KEEPS creeping forward and lets the one-wheel mixing below
				 * (correction from IR_GetCorrection) steer it smoothly back to centre.
				 * Only a FULL off-centre (+-2) or all-white (000) corner pivots now -
				 * this matches the graded design documented in ir_tracker.h. */
				in_pivot      = 0;                   // no tank pivot for a small drift
				base_speed    = LINE_DRIFT_SPEED;    // keep moving while steering (small curves flow)
				/* correction (IR PD term) is left as read -> gentle one-wheel mixing arcs back */
				center_count  = 0;
				was_lost      = 0;
				was_hard      = 0;
				lost_count    = 0;
				straight_corr = correction;          // seed the anti-sawtooth fade for when it re-centres
			}
			else if(ir_error != 0)                   // FULLY off to one side (001/100 = ~90deg) -> STEPPED tank pivot: small turn -> rebalance -> small turn ...
			{
				if(pivot_pause > 0)                  // between steps: balance on the spot, don't rotate yet (let the body recover)
				{
					in_pivot      = 0;
					base_speed    = 0;
					correction    = 0;
					center_count  = 0;
					was_lost      = 0;
					was_hard      = 1;
					did_hard_turn = 1;
				}
				else                                 // start ONE small pivot step
				{
					in_pivot        = 1;                 // HARD_TURN_TANK_PIVOT (absolute reverse on the inner wheel)
					pivot_dir       = (ir_error > 0) ? 1 : -1;
					pivot_seen_lost = 1;                 // exit at the next centred 010
					if(pivot_min_count == 0) pivot_min_count = PIVOT_STEP_CYCLES;  // small step length
					base_speed      = 0;                 // rotate on the spot, no forward creep
					center_count    = 0;
					was_lost        = 0;
					was_hard        = 1;
					did_hard_turn   = 1;                 // arms the post-turn backup
					lost_count      = 0;
					straight_corr   = 0;
				}
			}
			else                                     // centred (010)
			{
				lost_count = 0;

				/* OVER-ROTATE: a pivot that first reaches centre is usually still under-
				 * rotated (the middle sensor catches the new line before the heading has
				 * fully turned). Keep spinning PIVOT_OVERROTATE more cycles (one-shot) so
				 * the heading finishes the corner, instead of exiting and drifting off. */
				if(in_pivot && pivot_dir != 0 && !overrot_done)
				{
					pivot_min_count = PIVOT_OVERROTATE;
					overrot_done    = 1;
				}

				if(in_pivot && (pivot_min_count > 0 || !pivot_seen_lost) && pivot_dir != 0)
				{
					base_speed = 0;                  // still clearing the old line; rotate via PIVOT_OUTER/PIVOT_INNER, don't resume forward yet
					/* pivot_min_count is ticked centrally at the top each cycle */
					center_count = 0;
					was_lost = 0;
					was_hard = 1;
					straight_corr = 0;
				}
				else
				{
					in_pivot = 0;                    // fully centred -> pivot complete, resume forward motion
					pivot_min_count = 0;
					pivot_seen_lost = 0;
					pivot_dir = 0;
					overrot_done = 0;                // re-arm over-rotation for the next turn

					/* A real turn just finished and we're back at centre -> reverse a
					 * moment before driving on (the "turn-then-backup"); the END-of-block
					 * override does the actual reverse. */
					if(did_hard_turn && post_turn_back == 0)
					{
						post_turn_back = POST_TURN_BACK_CYCLES;
						did_hard_turn  = 0;
					}

					correction = straight_corr;      // gentle fading steering toward the last drift side (anti-sawtooth)
					straight_corr = straight_corr * 3 / 4;   // decay toward 0 the longer it stays centred

					if(was_lost || was_hard)         // re-acquiring after a lost (000) OR a turn: ease back up to speed
					{
						float lean = Angle_Balance - (Balance_Target + BALANCE_TRIM);
						if(lean < 0) lean = -lean;
						reached_center = 1;          // centred at least once -> later +-1 drifts become gentle fine-align
						if(lean >= SETTLE_ANGLE)     // still pitched (leaning) -> NOT settled; hold, or it'll drive forward and rush off
							center_count = 0;
						else if(center_count < CENTER_CONFIRM)
							center_count++;
						if(center_count >= CENTER_CONFIRM) { base_speed = LINE_BASE_SPEED; was_lost = 0; was_hard = 0; reached_center = 0; }  // centred AND upright -> resume
						else                                 base_speed = 0;                                             // not settled yet -> balance on spot
					}
					else                             // centre reached via a slight drift only -> full speed immediately
						base_speed = LINE_BASE_SPEED;
				}
			}

			/* Post-turn backup: while the reverse window is active (armed at the centred-
			 * resume point below, right after a real turn finishes), hold straight - no
			 * pivot, no steer. The reverse command itself is applied at the END of this
			 * block so the exit-brake can't overwrite it. */
			if(post_turn_back > 0)
			{
				in_pivot   = 0;
				correction = 0;
			}

			Move_Target_Cmd = base_speed;            // line-follower drives the forward command

			/* Active exit-brake: while still settling after a hard turn (was_hard),
			 * the balance loop drives forward to catch the turn-induced lean and the
			 * bot rushes off. Counter it by commanding REVERSE proportional to the
			 * measured forward translation (encoder sum), via the velocity loop
			 * (balance-consistent, won't pitch it over). Self-cancels once stopped. */
			/* TEMP DISABLED (no exit-brake): uncomment to re-enable post-turn braking.
			if(was_hard)
			{
				int esum  = Encoder_Left + Encoder_Right;
				int brake = 0;
				if(esum > EXIT_BRAKE_DEADZONE)                  // only a genuine forward rush -> brake; small wobble left alone so it can't ratchet backward
					brake = -(esum - EXIT_BRAKE_DEADZONE);
				if(brake < -EXIT_BRAKE_MAX) brake = -EXIT_BRAKE_MAX;
				Move_Target_Cmd = brake;
			}
			*/

			/* Climb DRIVE (solution a): once the seesaw is ARMED (a real climb confirmed),
			 * drive forward at SEESAW_CLIMB_SPEED instead of the slow line-follow base
			 * speed - a 30% slope can't be climbed at LINE_BASE_SPEED=1. Skipped while
			 * tipping (tip_brake) so the anti-dive can still cut forward at the crest. */
			if(Seesaw_Armed && !tip_brake)
				Move_Target_Cmd = SEESAW_CLIMB_SPEED;

			/* Post-turn backup has the FINAL say: reverse after a turn (overrides the
			 * forward base speed and the exit-brake which would otherwise zero it). */
			if(post_turn_back > 0)
			{
				Move_Target_Cmd = -POST_TURN_BACK_SPEED;
				post_turn_back--;
			}

			/* Sub-1 forward creep: LINE_BASE_SPEED is an integer (min 1). To go SLOWER
			 * than 1, let a FORWARD command through only 1 cycle in every LINE_SPEED_DIV
			 * (average forward speed = LINE_BASE_SPEED / LINE_SPEED_DIV). Reverse/stop
			 * are untouched. LINE_SPEED_DIV = 1 -> no change. */
			{
				static u8 spd_div = 0;
				if(++spd_div >= LINE_SPEED_DIV) spd_div = 0;
				if(Move_Target_Cmd > 0 && spd_div != 0)
					Move_Target_Cmd = 0;
			}
		}
		else
		{
			/* Descent steering trim (下坡往右 補償): while riding the tipped board
			 * down (pure balance, both wheels equal PWM) a motor mismatch makes it
			 * veer; inject a small fixed correction to cancel it. Off (0) otherwise. */
			correction = Seesaw_Descending ? DESCENT_STEER_TRIM : 0;
			/* Move_Target_Cmd is left as set by Set_Forward_Speed() (0 if never called) */

			/* CRITICAL: clear ALL line-follow/pivot state. The motor mixing keys off
			 * in_pivot, and the watchdog that releases it only runs inside the
			 * line-follow branch above - so a pivot latched mid-follow (or while
			 * being handled) used to survive a switch to pure-balance mode and pin
			 * the outside wheel at +PIVOT_OUTER forever: the bot could not stand
			 * still and surged forward. Reset everything so balance mode is clean. */
			in_pivot = 0;
			pivot_min_count = 0;
			pivot_seen_lost = 0;
			pivot_dir = 0;
			was_hard = 0;
			was_lost = 0;
			lost_count = 0;
			center_count = 0;
			straight_corr = 0;
			reached_center = 0;
			lost_from_pivot = 0;
		}

		/* Asymmetric speed change: ACCELERATE gradually (so resuming after a turn
		 * doesn't lurch to full speed and fling the bot off the track), but
		 * DECELERATE immediately (speed drops the instant a turn is commanded). */
		if(Turn_Off(Angle_Balance) == 1 || Flag_Stop == 1)
		{
			Move_Target_Cmd = 0;
			Move_Target = 0;
		}
		else if(Move_Target_Cmd < Move_Target)
			Move_Target = Move_Target_Cmd;   // decelerate immediately
		else
			Move_Target = PWM_Ramp(Move_Target_Cmd, Move_Target, MOVE_RAMP_STEP);  // accelerate gradually (one MOVE_RAMP_STEP per tick)

		/* Forward-lean feed-forward: bias the balance setpoint into the travel
		 * direction so the velocity loop does not have to drive the wheels out
		 * from under the (front-heavy) body. 0 gain leaves original behaviour. */
		Forward_Lean = (float)Move_Target * FORWARD_LEAN_GAIN;
		if(Forward_Lean >  FORWARD_LEAN_MAX) Forward_Lean =  FORWARD_LEAN_MAX;
		if(Forward_Lean < -FORWARD_LEAN_MAX) Forward_Lean = -FORWARD_LEAN_MAX;

		/* Dynamic mode select: FLAT line-following uses the line-follow setpoint & speed
		 * gain; climbing (armed) / descending / pure balance use the slope/balance ones.
		 * Set BEFORE Balance() and Velocity() so both loops use the matching values. */
		Track_Mode = (Flag_LineFollow && !Seesaw_Armed && !Seesaw_Descending) ? 1 : 0;
		Balance_Target = Track_Mode ? MIDDLE_ANGLE_TRACK : Middle_angle;

		Balance_Pwm  = Balance(Angle_Balance, Gyro_Balance);     // Upright PD control (uses Forward_Lean)

		Velocity_Pwm = Velocity(Encoder_Left, Encoder_Right);    // Speed P(I) control

		g_balance_pwm  = Balance_Pwm;   // diagnostic snapshots for the OLED breakdown
		g_velocity_pwm = Velocity_Pwm;

		/* NOTE: the speed loop is intentionally LEFT ACTIVE during the in-place pivot.
		 * The pivot is a SYMMETRIC counter-rotation (one wheel +spin, the other
		 * -spin), so Encoder_Left + Encoder_Right ~= 0 -> the speed loop does NOT
		 * fight the spin. What it DOES see is any real forward translation, e.g. the
		 * momentum the bot carries into a turn - and it brakes that, so the bot can
		 * stop coasting and rotate in place instead of sliding forward first. (It was
		 * zeroed here back when the pivot was a ONE-wheel reverse, whose asymmetric
		 * encoders made the loop surge forward.) */

		/* Speed brake - ONLY while line-following (Flag_LineFollow). It brakes the
		 * forward translation when the body leans, taming the line-follow forward surge
		 * (the working 循跡 build relies on this; disabling it made the bot run away and
		 * fall while tracking). It is gated OFF in pure balance so it can't fight the
		 * balance catch at a standstill. Force lowered from x3/±250 to x2/±200 per the
		 * "煞車力道小一點點" request - raise back toward x3/±250 if surge returns, lower
		 * (x1/±120) if it feels grabby. Also OFF while CLIMBING (Seesaw_Armed): on a
		 * slope the lean is large and sustained, which would otherwise brake the climb. */
		if(Track_Mode)   // flat line-following only (= Flag_LineFollow & not climbing/descending)
		{
			static u8 brake_active = 0;   // hysteresis latch: brakes between the 1.4/0.7 deg thresholds
			float angle_error;
			int   encoder_sum;

			angle_error = Angle_Balance - (Balance_Target + BALANCE_TRIM + Forward_Lean);
			if(angle_error < 0) angle_error = -angle_error;
			encoder_sum = Encoder_Left + Encoder_Right;
			if(angle_error > 1.4f)       brake_active = 1;
			else if(angle_error < 0.7f)  brake_active = 0;

			if(brake_active && myabs(encoder_sum) > 3)
				brake_target = PWM_Limit(-encoder_sum * 2, 200, -200);   // x2 gain, ±200 cap (original was x3/±250)
			else
				brake_target = 0;
		}
		else
			brake_target = 0;

		brake_pwm = (brake_pwm * 7 + brake_target) / 8;
		early_brake = brake_pwm;

		{
			int common_pwm = Balance_Pwm + Velocity_Pwm + early_brake;
#ifdef STEER_PIVOT_ONE_WHEEL
			int inside_corr = correction * STEER_INNER_PERCENT / 100;
			int outside_corr = correction * STEER_OUTER_PERCENT / 100;
#endif

			/* Anti-dive: while an armed climb is collapsing (the board is tipping),
			 * cap how hard the loop may drive FORWARD. As the body pitches back past
			 * the setpoint the balance term turns strongly positive (forward) trying
			 * to get the wheels back under the body - on a tipping seesaw that is the
			 * forward launch ("向前撲倒"). Clamping the forward side lets it ride the
			 * board down instead of flinging the wheels out. Reverse (pull-back) drive
			 * is left untouched. */
			if(tip_brake && common_pwm > SEESAW_FWD_CLAMP) common_pwm = SEESAW_FWD_CLAMP;
#ifdef STEER_PIVOT_ONE_WHEEL

#ifdef HARD_TURN_TANK_PIVOT
			if(in_pivot)   // the 000-corner pivot (started by the lost path); +-1/+-2 never spin this - they use the gentle mixing below
			{
				/* Hard turn: GUARANTEED one-wheel-forward / one-wheel-backward.
				 * The OUTSIDE wheel runs common_pwm + PIVOT_OUTER (it still carries
				 * the balance term). The INSIDE wheel is commanded an ABSOLUTE
				 * reverse of -PIVOT_INNER, NOT an offset off common_pwm.
				 *
				 * Why absolute: entering a corner the body leans forward, so the
				 * balance term is large positive (~500 PWM per degree of lean - 3 deg
				 * = +1500). An offset of common_pwm - PIVOT_INNER then stays POSITIVE,
				 * i.e. BOTH wheels drove forward and the bot surged through the corner
				 * with no visible wheel reversal. Pinning the inside wheel to a real
				 * reverse makes the pivot actually counter-rotate every time.
				 * (Trade-off: only the outside wheel balances during the pivot - if it
				 * now falls forward in corners, lower PIVOT_INNER or raise PIVOT_OUTER.)
				 * turn_sign follows the latched pivot direction; if it turns the WRONG
				 * way, negate turn_sign. */
				/* DIRECTION - settled by the clean field test AFTER the encoder fix
				 * (the only trustworthy one; every earlier direction experiment ran
				 * with broken encoder feedback and was misleading): error > 0
				 * (001/011 side) -> LEFT wheel reverses, RIGHT wheel drives forward.
				 * The opposite assignment made every turn steer away from the line
				 * ("轉彎全相反"). Matches the gentle-steer mixing below. */
				int turn_sign = pivot_dir;
				if(turn_sign > 0)
				{
					Motor_Left  = -(PIVOT_INNER * PIVOT_GAIN_ERRPOS / 100);             // fixed slow reverse
					Motor_Right = common_pwm + (PIVOT_OUTER * PIVOT_GAIN_ERRPOS / 100); // forward + balance
				}
				else if(turn_sign < 0)
				{
					Motor_Left  = common_pwm + (PIVOT_OUTER * PIVOT_GAIN_ERRNEG / 100); // forward + balance
					Motor_Right = -(PIVOT_INNER * PIVOT_GAIN_ERRNEG / 100);             // fixed slow reverse
				}
				else
				{
					Motor_Left  = common_pwm;
					Motor_Right = common_pwm;
				}
			}
			else
#endif
			/* Gentle steering, SAME yaw direction as the pivot above: correction > 0
			 * (001/011 side) -> left wheel slows (inside), right wheel speeds up
			 * (outside). Confirmed by the post-encoder-fix field test - the opposite
			 * assignment reversed every turn. */
			if(correction > 0)
			{
				Motor_Left  = common_pwm - inside_corr;
				Motor_Right = common_pwm + outside_corr;
			}
			else if(correction < 0)
			{
				Motor_Left  = common_pwm - outside_corr;  // outside_corr is negative -> speeds the left wheel
				Motor_Right = common_pwm + inside_corr;   // inside_corr is negative -> slows the right wheel
			}
			else
			{
				Motor_Left  = common_pwm;
				Motor_Right = common_pwm;
			}
#else
			Motor_Left  = common_pwm - correction;  // direction matched to the pivot orientation above (post-encoder-fix field test)
			Motor_Right = common_pwm + correction;
#endif
		}

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
	float Balance_Kp = 300, Balance_Kd = 1.2;
	float Angle_bias, Gyro_bias;
	int   balance;

	Angle_bias = (Balance_Target + BALANCE_TRIM + Forward_Lean) - Angle;   
	Gyro_bias  = 0 - (Gyro - Gyro_Zero);          
	balance    = Balance_Kp * Angle_bias + Gyro_bias * Balance_Kd;   

	/* --- ????! --- */
	balance = PWM_Limit(balance, 6900, -6900);
	
	return balance;
}



/**************************************************************************
Function: Velocity PI control - computes PWM correction to maintain
          zero average wheel speed (robot stays on spot)
Input   : encoder_left  - left wheel encoder count (this 10ms period)
          encoder_right - right wheel encoder count (this 10ms period)
Output  : Velocity control PWM value
**************************************************************************/
int Velocity(int encoder_left, int encoder_right)
{
#ifdef DEBUG_BALANCE_ONLY
	float Velocity_Kp = -100,    Velocity_Ki = -0.4;             // DEBUG: speed loop truly OFF -> pure balance (isolation test). (Was -160/-0.8 by mistake, i.e. NOT off.)
#else
	/* Mode-dependent proportional gain: lighter while FLAT line-following
	 * (VELOCITY_KP_TRACK, the proven 循跡 value), heavier on a slope / in balance
	 * (VELOCITY_KP_BAL). Track_Mode is set each cycle in the ISR before this runs. */
	float Velocity_Kp = Track_Mode ? VELOCITY_KP_TRACK : VELOCITY_KP_BAL;
	/* Mode-dependent integral gain: GENTLE when standing (VELOCITY_KI) so the
	 * standstill balance stays smooth (the value proven under DEBUG); STRONG when
	 * driving/climbing (VELOCITY_KI_CLIMB) so there is real sustained uphill push. */
	float Velocity_Ki = (Move_Target != 0) ? VELOCITY_KI_CLIMB : VELOCITY_KI;
#endif
	static float velocity, Encoder_Least, Encoder_bias;
	static float Encoder_Integral;

	if(Velocity_Reset_Req)   /* seesaw just tipped: dump the wound-up climb push before it launches the bot down the new downhill */
	{
		Velocity_Reset_Req = 0;
		Encoder_Integral   = 0;
		Encoder_bias       = 0;
	}

	/* Speed error = target - actual speed. Move_Target is 0 for stationary
	 * balance, or the line-follow forward speed when tracking. */
	Encoder_Least   = Move_Target - (encoder_left + encoder_right);

	/* First-order low-pass filter: 80% previous + 20% new */
	Encoder_bias   *= 0.8;
	Encoder_bias   += Encoder_Least * 0.2;

	/* Integrate filtered error (10ms period). Plain continuous integration - the
	 * original, known-good behaviour. (Conditional/frozen-integration anti-windup
	 * schemes were tried to soften a hard-push return but each introduced a worse
	 * disturbance mode - a pumped oscillation, or losing the force that settles the
	 * swing - so they were removed. The continuous integral provides a continuous
	 * restoring force that helps the swing converge.) */
	Encoder_Integral += Encoder_bias;
	if(Encoder_Integral >  4000) Encoder_Integral =  4000;   // clamp kept full so the flat-ground hold settles smoothly (a small clamp starved it -> drift/hunt)
	if(Encoder_Integral < -4000) Encoder_Integral = -4000;

	/* Riding the tipped board DOWN: hold the position-integral at 0. Otherwise it
	 * winds up as the bot rolls down the slope and drives the wheels BACKWARD to
	 * "pull its position back" - the unwanted reverse during descent. Encoder_bias
	 * (the speed-damping P term) is kept, so the roll-down stays gentle. This only
	 * applies in the descent window; normal balance/position-hold is untouched. */
	if(Seesaw_Descending) Encoder_Integral = 0;

	velocity = -Encoder_bias * Velocity_Kp - Encoder_Integral * Velocity_Ki;

	/* Reset integrator when motors are disabled/stopped, OR while line-following with
	 * no forward command (turn/settle/lost). In PURE BALANCE the integral is left to
	 * accumulate (full clamp) for a smooth, drift-free flat-ground position hold - this
	 * is the original, known-good behaviour. (Attempts to also clear/cap it here to
	 * pre-empt a slope dash instead broke flat balance: clearing bias killed the speed
	 * damping -> "往傾斜的地方衝"; clearing the integral killed the hold -> "往後退"; a
	 * small cap starved the hold -> front/back hunting "前後飄". The slope-dash is
	 * handled where it actually occurs - the seesaw tip sets Velocity_Reset_Req.) */
	if(Turn_Off(Angle_Balance) == 1 || Flag_Stop == 1 || (Flag_LineFollow && Move_Target == 0))
	{
		Encoder_Integral = 0;
		Encoder_bias     = 0;   // also clear the low-pass memory so a pivot's "moving backward" reading doesn't blip forward on exit
	}

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

	g_pwm_left  = motor_left;    // diagnostic snapshot (signed) for the OLED
	g_pwm_right = motor_right;

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

/**************************************************************************
Function: Toggle the robot stop state when the key is pressed
Input   : none
Output  : none
**************************************************************************/
/* Double-click window in g_isr_count ticks (200Hz ISR -> 5ms each). 80 = 400ms.
 * REAL-TIME based on purpose: the old version counted main-loop iterations, so
 * the window's real duration depended on how fast oled_show() runs - enabling
 * compiler -O3 sped the loop up and shrank the window to ~250ms, making the
 * double-click nearly impossible to hit. ISR ticks don't care about loop speed. */
#define DBL_CLICK_TICKS 80

void Key(void)
{
	static u8  click_pending = 0;   // waiting to see if a 2nd click arrives
	static u32 click_isr     = 0;   // g_isr_count at the 1st click
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
			click_isr = g_isr_count;
		}
		return;
	}

	if(click_pending && (u32)(g_isr_count - click_isr) >= DBL_CLICK_TICKS)
	{                                   // window expired with no 2nd click -> SINGLE click
		if(Flag_Stop == 0 || Balance_Ready)
			Flag_Stop = !Flag_Stop;     // only allow ON after calibration is ready
		click_pending = 0;
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
	if(angle < -50 || angle > 50 || Flag_Stop == 1)
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
	Gyro_Balance  = gyro[1];      // hardware-confirmed damping axis; gyro[1] caused immediate fall
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
Function: Command a forward-speed target. The ISR ramps Move_Target toward
          this each 10ms tick (MOVE_RAMP_STEP), so callers can step it freely
          without lurching a front-heavy body. Positive = forward; 0 = hold.
          Ignored while line-follow is active (the line-follower owns the
          command then).
Input   : speed - desired forward speed (encoder counts / 10ms, summed L+R)
Output  : none
**************************************************************************/
void Set_Forward_Speed(int speed)
{
	Move_Target_Cmd = speed;
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

/* Current operating mode (for the OLED display).
 * 0 = BAL   : pure balance (line-follow off)
 * 1 = TRACK : flat line-following (using the line-follow setpoint/gains)
 * 2 = CLIMB : up-slope confirmed (armed) - using the balance/slope setpoint/gains
 * 3 = DOWN  : riding the tipped board down (post-tip descent) */
u8 Control_GetMode(void)
{
	if(Seesaw_Descending) return 3;   // DOWN
	if(Seesaw_Armed)      return 2;   // CLIMB
	if(Flag_LineFollow)   return 1;   // TRACK
	return 0;                          // BAL
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
