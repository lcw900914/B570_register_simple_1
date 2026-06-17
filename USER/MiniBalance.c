/**
 * MiniBalance.c - Main entry point for self-balancing robot
 *
 * Hardware:  STM32F103C8T6 + S28A adapter board
 * Sensors:   MPU6050 (DMP) on PB8/PB9 (software I2C)
 * Motors:    TB6612 driven by TIM1 PWM (PWMA=PA8, PWMB=PA11)
 *            Direction: AIN1=PB14, AIN2=PB15, BIN1=PB13, BIN2=PB12
 * Encoders:  Left  = TIM3 (PA6/PA7)
 *            Right = TIM4 (PB6/PB7)
 * INT pin:   MPU6050 -> PA0 (EXTI0, falling edge); moved from dead PA12
 * Key:       PA5 (active low)
 *
 * Control loop runs inside EXTI15_10_IRQHandler (control.c),
 * triggered by MPU6050 INT at 200 Hz -> actual PID runs at 100 Hz
 * (every other interrupt, gated by Flag_Target -> dt = 10ms).
 */

#include "sys.h"
#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "exti.h"
#include "key.h"
#include "control.h"
#include "oled.h"
#include "show.h"
#include "ir_tracker.h"

// Globals - declared extern in sys.h, defined here
u8    Flag_Stop      = 1;   // start with motors OFF for safe bring-up; press key to enable
float Angle_Balance  = 0;   // Current tilt angle from DMP (degrees)
float Gyro_Balance   = 0;   // Current angular rate from gyro (degrees/s)
int   Encoder_Left   = 0;   // Left  wheel encoder count (updated in ISR)
int   Encoder_Right  = 0;   // Right wheel encoder count (updated in ISR)

int main(void)
{
    Stm32_Clock_Init(9);
    delay_init(72);
    uart_init(72, 115200);

    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    IIC_Init();

    MiniBalance_PWM_Init(7199, 0);
    MiniBalance_Motor_Init();

    Encoder_Init_TIM3();
    Encoder_Init_TIM4();

    KEY_Init();

    IR_Init();       // 3-channel IR line tracker on PB1/PA1/PA2 (input pull-up)

    OLED_Init();     // OLED uses software SPI (PB3/PA15/PB5/PB4), powered from the 12V rail
    OLED_Clear();

    DMP_Init();      // Initialize MPU6050 DMP (200 Hz quaternion output)

    EXTI_Init();     // Enable MPU6050 INT -> PA0 EXTI0; this is what starts the balance loop

    MPU_ClearINT();  // release the latched INT once so the first falling edge can fire the ISR (Get_Angle keeps clearing it thereafter)

    /* Independent watchdog (IWDG): the entire balance loop lives in the EXTI ISR,
     * and the motors hold their last PWM whenever that loop stops. If the CPU ever
     * gets pinned (interrupt storm) or the main loop hangs (e.g. OLED SPI), the
     * IWDG is no longer fed and the chip hardware-resets after ~1s. After reset
     * Flag_Stop=1 (motors off at boot), so a frozen bot STOPS instead of driving
     * on with a stale command. Fed once per main-loop pass below.
     * LSI ~40kHz, /64 prescaler -> 625Hz tick; RLR=625 -> ~1.0s timeout. */
    IWDG->KR  = 0x5555;   // enable write access to PR/RLR
    IWDG->PR  = 4;        // prescaler = /64
    IWDG->RLR = 625;      // ~1.0s timeout
    IWDG->KR  = 0xAAAA;   // reload counter
    IWDG->KR  = 0xCCCC;   // start the watchdog

    while(1)
    {
        IWDG->KR = 0xAAAA;   // feed the watchdog (only here -> a frozen loop won't feed it -> reset)

        /* Control-loop liveness failsafe: g_isr_count ticks on every EXTI ISR
         * (200Hz). If it stops advancing while the main loop is still alive (the
         * MPU stopped issuing INT, FIFO stall, etc.), the balance loop is DEAD but
         * the motors are still energised at their last PWM -> the bot drives on
         * with no angle update. Detect the stall and cut the motors hard. */
        {
            static u32 prev_isr = 0;
            static u8  started = 0, stall = 0;
            if(g_isr_count != prev_isr) { prev_isr = g_isr_count; stall = 0; started = 1; }
            else if(started && ++stall >= 3)   // ~3 passes (>=150ms) with NO control ISR -> balance loop is dead
            {
                /* The EXTI ISR stopped firing - typically the MPU INT latched LOW after
                 * a FIFO overflow, so no new falling edge is produced and the loop dies.
                 * Recover it instead of giving up:
                 *   1) cut the H-bridge so the motors can't keep driving on the last PWM
                 *      (the "freeze then rush" failure), then
                 *   2) re-clear the MPU INT to release the stuck line and revive the IRQ
                 *      train so the bot comes back by itself.
                 * Deliberately does NOT latch Flag_Stop: an earlier version forced it to 1
                 * here, and because this runs EVERY pass while stalled it kept overriding
                 * the key, so the bot got stuck OFF and could never be turned back ON. */
                Set_Pwm(0, 0);                 // motors safe while the loop is dead
                MPU_ClearINT();                // release a stuck/latched INT so the ISR can fire again
                stall = 0;                     // give it a few passes to revive before retrying
            }
        }

        Key();   // button polled here: the EXTI ISR that used to host Key() is unreliable

        // Status on the OLED (no UART needed -> works while running on 12V).
        // Hold the robot at its real balance point and read "Angle": that value
        // is what Middle_angle should be set to.
        oled_show();
        delay_ms(50);
    }
}
