#include "sys.h"

/**************************************************************************
Function: Set the NVIC vector table base address and offset
Input   : NVIC_VectTab - base address (flash or RAM)
          Offset       - byte offset (must be 128-byte aligned)
Output  : none
**************************************************************************/
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{
	/* Write to VTOR: identifies whether vectors are in CODE or RAM region */
	SCB->VTOR = NVIC_VectTab | (Offset & (u32)0x1FFFFF80);
}

/**************************************************************************
Function: Configure the NVIC interrupt priority grouping
Input   : NVIC_Group - group number 0~4
          Group | Preempt bits | Sub-priority bits
          ------+-------------+------------------
            0   |      0      |        4
            1   |      1      |        3
            2   |      2      |        2
            3   |      3      |        1
            4   |      4      |        0
Output  : none
**************************************************************************/
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
	u32 temp, temp1;
	temp1  = (~NVIC_Group) & 0x07;  // Invert lower 3 bits
	temp1 <<= 8;
	temp   = SCB->AIRCR;             // Read current value
	temp  &= 0X0000F8FF;             // Clear priority group field
	temp  |= 0X05FA0000;             // Write access key
	temp  |= temp1;
	SCB->AIRCR = temp;               // Write new group setting
}

/**************************************************************************
Function: Configure and enable one NVIC interrupt channel
Input   : NVIC_PreemptionPriority - preemption priority level
          NVIC_SubPriority        - sub-priority level
          NVIC_Channel            - IRQ number
          NVIC_Group              - priority grouping (0~4)
Output  : none
**************************************************************************/
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority,
                  u8 NVIC_Channel, u8 NVIC_Group)
{
	u32 temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group);            // Set priority group
	temp  = NVIC_PreemptionPriority << (4 - NVIC_Group);
	temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
	temp &= 0xf;
	NVIC->ISER[NVIC_Channel/32] |= (1 << NVIC_Channel%32);  // Enable IRQ
	NVIC->IP[NVIC_Channel]      |= temp << 4;                // Set priority
}

/**************************************************************************
Function: Configure an external interrupt line for one GPIO pin
          This function handles only one pin per call.
          It automatically enables the corresponding interrupt line.
Input   : GPIOx - port index (GPIO_A=0 ... GPIO_G=6)
          BITx  - pin number (0~15)
          TRIM  - trigger mode: FTIR=falling, RTIR=rising, 3=both
Output  : none
**************************************************************************/
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM)
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR   = BITx / 4;      // Which EXTICR register
	EXTOFFSET = (BITx % 4) * 4;
	RCC->APB2ENR |= 0x01;      // Enable AFIO clock
	AFIO->EXTICR[EXTADDR] &= ~(0x000F << EXTOFFSET);   // Clear old mapping
	AFIO->EXTICR[EXTADDR] |=   GPIOx  << EXTOFFSET;    // Map EXTI line to GPIO port
	EXTI->IMR |= 1 << BITx;    // Unmask interrupt on this line
	if(TRIM & 0x01) EXTI->FTSR |= 1 << BITx;  // Enable falling-edge trigger
	if(TRIM & 0x02) EXTI->RTSR |= 1 << BITx;  // Enable rising-edge trigger
}

/**************************************************************************
Function: Reset all RCC clock registers to their default state and
          set the NVIC vector table location
Output  : none
Note    : Do not call this while running from UART - it will disable peripherals.
**************************************************************************/
void MYRCC_DeInit(void)
{
	RCC->APB1RSTR = 0x00000000;  // Release APB1 peripherals from reset
	RCC->APB2RSTR = 0x00000000;

	RCC->AHBENR   = 0x00000014;  // Keep SRAM and Flash clocks; disable others
	RCC->APB2ENR  = 0x00000000;  // Disable all APB2 peripheral clocks
	RCC->APB1ENR  = 0x00000000;  // Disable all APB1 peripheral clocks

	RCC->CR   |= 0x00000001;     // Enable internal HSI oscillator
	RCC->CFGR &= 0xF8FF0000;     // Reset SW, HPRE, PPRE1, PPRE2, ADCPRE, MCO
	RCC->CR   &= 0xFEF6FFFF;     // Disable HSE, CSS, PLL
	RCC->CR   &= 0xFFFBFFFF;     // Clear HSE bypass
	RCC->CFGR &= 0xFF80FFFF;     // Reset PLL source and multiplier
	RCC->CIR   = 0x00000000;     // Disable all clock interrupts

	/* Set vector table location */
#ifdef VECT_TAB_RAM
	MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else
	MY_NVIC_SetVectorTable(0x08000000, 0x0);
#endif
}

/* Execute WFI (Wait For Interrupt) - THUMB mode only */
__asm void WFI_SET(void)
{
	WFI;
}

/* Disable all interrupts (sets PRIMASK) */
__asm void INTX_DISABLE(void)
{
	CPSID I;
}

/* Enable all interrupts (clears PRIMASK) */
__asm void INTX_ENABLE(void)
{
	CPSIE I;
}

/* Set the Main Stack Pointer to addr */
__asm void MSR_MSP(u32 addr)
{
	MSR MSP, r0  // set Main Stack value
	BX r14
}

/**************************************************************************
Function: Enter standby (deep sleep) mode. Wakes on WKUP pin event.
Input   : none
Output  : none
**************************************************************************/
void Sys_Standby(void)
{
	SCB->SCR   |= 1<<2;        // Set SLEEPDEEP bit
	RCC->APB1ENR |= 1<<28;     // Enable power interface clock
	PWR->CSR   |= 1<<8;        // Enable WKUP pin wakeup
	PWR->CR    |= 1<<2;        // Clear wakeup flag
	PWR->CR    |= 1<<1;        // Select standby mode (PDDS)
	WFI_SET();                  // Enter standby
}

/**************************************************************************
Function: Perform a software system reset
Input   : none
Output  : none
**************************************************************************/
void Sys_Soft_Reset(void)
{
	SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}

/**************************************************************************
Function: Configure JTAG/SWD debug interface
Input   : mode - 0x00=both JTAG+SWD, 0x01=SWD only, 0x02=both disabled
Output  : none
**************************************************************************/
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp  = mode;
	temp <<= 25;
	RCC->APB2ENR |= 1<<0;        // Enable AFIO clock
	AFIO->MAPR   &= 0XF8FFFFFF;  // Clear SWJ_CFG[2:0] in MAPR
	AFIO->MAPR   |= temp;        // Write new JTAG/SWD mode
}

/**************************************************************************
Function: Initialize the system clock using the PLL
          External HSE crystal -> PLL -> SYSCLK
          APB1 = HCLK/2 (max 36MHz), APB2 = HCLK/1, AHB = HCLK/1
Input   : PLL - PLL multiplier value (2~16); e.g. 9 -> 72MHz with 8MHz HSE
Output  : none
**************************************************************************/
void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp = 0;
	MYRCC_DeInit();              // Reset all clocks

	RCC->CR |= 0x00010000;       // Enable HSE oscillator
	while(!(RCC->CR >> 17));     // Wait for HSE to stabilize

	RCC->CFGR  = 0X00000400;     // APB1=HCLK/2, APB2=HCLK/1, AHB=HCLK/1
	PLL -= 2;                    // Adjust: register value = multiplier - 2
	RCC->CFGR |= PLL << 18;      // Set PLL multiplier
	RCC->CFGR |= 1 << 16;        // PLL source: HSE
	FLASH->ACR |= 0x32;          // Flash: 2 wait states (required at 72MHz)

	RCC->CR |= 0x01000000;       // Enable PLL
	while(!(RCC->CR >> 25));     // Wait for PLL to lock

	RCC->CFGR |= 0x00000002;     // Select PLL as system clock
	while(temp != 0x02)          // Wait until PLL is confirmed as system clock
	{
		temp  = RCC->CFGR >> 2;
		temp &= 0x03;
	}
}
