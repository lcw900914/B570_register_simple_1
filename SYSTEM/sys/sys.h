#ifndef __SYS_H
#define __SYS_H
#include <stm32f10x.h>

/* Set to 1 to enable uCOS-II support, 0 to disable */
#define SYSTEM_SUPPORT_UCOS  0

/* Cortex-M3 bit-band access macros.
   Allows atomic read-modify-write on individual bits in peripheral and SRAM regions.
   Reference: Cortex-M3 Technical Reference Manual, pages 87-92. */
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)        *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

/* GPIO Output Data Register base addresses */
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C

/* GPIO Input Data Register base addresses */
#define GPIOA_IDR_Addr    (GPIOA_BASE+8)  //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8)  //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8)  //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8)  //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8)  //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8)  //0x40011E08

/* Single-bit GPIO access macros (output and input), valid for pin n < 16 */
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr, n)  // PA output bit
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr, n)  // PA input bit

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr, n)  // PB output bit
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr, n)  // PB input bit

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr, n)  // PC output bit
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr, n)  // PC input bit

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr, n)  // PD output bit
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr, n)  // PD input bit

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr, n)  // PE output bit
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr, n)  // PE input bit

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr, n)  // PF output bit
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr, n)  // PF input bit

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr, n)  // PG output bit
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr, n)  // PG input bit

/* GPIO port index constants used by Ex_NVIC_Config */
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6

/* External interrupt trigger modes for Ex_NVIC_Config */
#define FTIR   1   // Falling edge trigger
#define RTIR   2   // Rising edge trigger

#include "delay.h"
#include "key.h"
#include "oled.h"
#include "usart.h"
#include "motor.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "show.h"
#include "exti.h"
#include "control.h"

/* JTAG/SWD debug port modes */
#define JTAG_SWD_DISABLE   0X02  // Disable both JTAG and SWD
#define SWD_ENABLE         0X01  // Enable SWD only
#define JTAG_SWD_ENABLE    0X00  // Enable both JTAG and SWD

/* Global variables shared across modules */
extern u8    Flag_Stop;                         // 1=motors stopped, 0=running
extern float Angle_Balance, Gyro_Balance;       // Current tilt angle and angular rate
extern int   Encoder_Left, Encoder_Right;       // Encoder counts (left and right wheels)

/* System functions */
void Stm32_Clock_Init(u8 PLL);
void Sys_Soft_Reset(void);
void Sys_Standby(void);
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM);
void JTAG_Set(u8 mode);

/* Inline assembly utility functions */
void WFI_SET(void);       // Execute WFI (Wait For Interrupt) instruction
void INTX_DISABLE(void);  // Disable all interrupts (CPSID I)
void INTX_ENABLE(void);   // Enable all interrupts (CPSIE I)
void MSR_MSP(u32 addr);   // Set the Main Stack Pointer

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#endif
