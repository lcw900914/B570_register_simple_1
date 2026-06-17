#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"

/* Frame buffer: 128 columns x 8 pages (each page = 8 rows, total 64 rows) */
u8 OLED_GRAM[128][8];

/**************************************************************************
Function: Flush the frame buffer to the OLED display
Input   : none
Output  : none
**************************************************************************/
void OLED_Refresh_Gram(void)
{
	u8 i, n;
	for(i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xb0 + i, OLED_CMD);  // Set page address (0~7)
		OLED_WR_Byte(0x00, OLED_CMD);      // Set column address low nibble
		OLED_WR_Byte(0x10, OLED_CMD);      // Set column address high nibble
		for(n = 0; n < 128; n++)
			OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
	}
}

/**************************************************************************
Function: Write one byte to the OLED via software SPI
Input   : dat - byte to write, cmd - 0=command, 1=data
Output  : none
**************************************************************************/
void OLED_WR_Byte(u8 dat, u8 cmd)
{
	u8 i;
	if(cmd)
		OLED_RS_Set();  // Data mode
	else
		OLED_RS_Clr();  // Command mode
	for(i = 0; i < 8; i++)
	{
		OLED_SCLK_Clr();
		if(dat & 0x80)
			OLED_SDIN_Set();
		else
			OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat <<= 1;
	}
	OLED_RS_Set();
}

/**************************************************************************
Function: Turn on the OLED display (enable charge pump and display)
Input   : none
Output  : none
**************************************************************************/
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D, OLED_CMD);  // Set charge pump
	OLED_WR_Byte(0X14, OLED_CMD);  // Charge pump ON
	OLED_WR_Byte(0XAF, OLED_CMD);  // Display ON
}

/**************************************************************************
Function: Turn off the OLED display (disable charge pump and display)
Input   : none
Output  : none
**************************************************************************/
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D, OLED_CMD);  // Set charge pump
	OLED_WR_Byte(0X10, OLED_CMD);  // Charge pump OFF
	OLED_WR_Byte(0XAE, OLED_CMD);  // Display OFF
}

/**************************************************************************
Function: Clear the display (fill frame buffer with 0 and refresh)
Input   : none
Output  : none
**************************************************************************/
void OLED_Clear(void)
{
	u8 i, n;
	for(i = 0; i < 8; i++)
		for(n = 0; n < 128; n++)
			OLED_GRAM[n][i] = 0X00;
	OLED_Refresh_Gram();
}

/**************************************************************************
Function: Draw or erase a single pixel in the frame buffer
Input   : x, y - pixel coordinates; t - 1=draw, 0=erase
Output  : none
**************************************************************************/
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
	u8 pos, bx, temp = 0;
	if(x > 127 || y > 63) return;  // Out of bounds
	pos  = 7 - y / 8;
	bx   = y % 8;
	temp = 1 << (7 - bx);
	if(t) OLED_GRAM[x][pos] |=  temp;
	else  OLED_GRAM[x][pos] &= ~temp;
}

/**************************************************************************
Function: Display a single ASCII character at the specified position
Input   : x, y - top-left coordinate; chr - ASCII character;
          size - font size (12 or 16); mode - 1=normal, 0=inverted
Output  : none
**************************************************************************/
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{
	u8 temp, t, t1;
	u8 y0 = y;
	chr = chr - ' ';  // Get offset from space character
	for(t = 0; t < size; t++)
	{
		if(size == 12)  temp = oled_asc2_1206[chr][t];  // 12x6 font
		else            temp = oled_asc2_1608[chr][t];  // 16x8 font
		for(t1 = 0; t1 < 8; t1++)
		{
			if(temp & 0x80)  OLED_DrawPoint(x, y,  mode);
			else             OLED_DrawPoint(x, y, !mode);
			temp <<= 1;
			y++;
			if((y - y0) == size)
			{
				y = y0;
				x++;
				break;
			}
		}
	}
}

/**************************************************************************
Function: Compute m raised to the power n (integer)
Input   : m - base, n - exponent
Output  : result
**************************************************************************/
u32 oled_pow(u8 m, u8 n)
{
	u32 result = 1;
	while(n--) result *= m;
	return result;
}

/**************************************************************************
Function: Display an unsigned decimal number at the specified position
Input   : x, y - starting coordinate; num - value (0~4294967295);
          len - number of digits to display; size - font size
Output  : none
**************************************************************************/
void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size)
{
	u8 t, temp;
	u8 enshow = 0;
	for(t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if(enshow == 0 && t < (len - 1))
		{
			if(temp == 0)
			{
				OLED_ShowChar(x + (size/2)*t, y, ' ', size, 1);
				continue;
			} else enshow = 1;
		}
		OLED_ShowChar(x + (size/2)*t, y, temp + '0', size, 1);
	}
}

/**************************************************************************
Function: Display a null-terminated ASCII string
Input   : x, y - starting coordinate; p - pointer to string
Output  : none
**************************************************************************/
/* Uses 16-pixel-wide font (12pt) */
void OLED_ShowString(u8 x, u8 y, const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
	while(*p != '\0')
	{
		if(x > MAX_CHAR_POSX) { x = 0; y += 16; }
		if(y > MAX_CHAR_POSY) { y = x = 0; OLED_Clear(); }
		OLED_ShowChar(x, y, *p, 12, 1);
		x += 8;
		p++;
	}
}

/**************************************************************************
Function: Re-send the SSD1306 configuration command sequence (no hardware
          reset, no 100ms delay, no frame clear). Cheap (~25 command bytes)
          so it can be called periodically from the main loop to RESYNC a
          controller whose config registers were corrupted by motor/EMI noise
          on the bit-banged SPI lines - the cause of the "OLED frozen / garbled
          while the board keeps running" failure. The per-frame refresh already
          re-asserts addressing + display-on, but the addressing MODE, segment
          remap, multiplex, contrast etc. are only set here, so without this
          periodic re-send a glitch in them stays until power-cycle.
**************************************************************************/
void OLED_Config(void)
{
	OLED_WR_Byte(0xAE, OLED_CMD);  // Display OFF
	OLED_WR_Byte(0xD5, OLED_CMD);  // Set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(80,   OLED_CMD);  // [3:0]=divide ratio; [7:4]=oscillator freq
	OLED_WR_Byte(0xA8, OLED_CMD);  // Set multiplex ratio
	OLED_WR_Byte(0X3F, OLED_CMD);  // 1/64 duty (default 0x3F)
	OLED_WR_Byte(0xD3, OLED_CMD);  // Set display offset
	OLED_WR_Byte(0X00, OLED_CMD);  // No offset (default)

	OLED_WR_Byte(0x40, OLED_CMD);  // Set display start line = 0

	OLED_WR_Byte(0x8D, OLED_CMD);  // Charge pump setting
	OLED_WR_Byte(0x14, OLED_CMD);  // Charge pump enabled
	OLED_WR_Byte(0x20, OLED_CMD);  // Set memory addressing mode
	OLED_WR_Byte(0x02, OLED_CMD);  // Page addressing mode (default)
	OLED_WR_Byte(0xA1, OLED_CMD);  // Set segment remap: col 127 -> SEG0
	OLED_WR_Byte(0xC0, OLED_CMD);  // Set COM scan direction: normal (COM0 to COM[N-1])
	OLED_WR_Byte(0xDA, OLED_CMD);  // Set COM pins hardware configuration
	OLED_WR_Byte(0x12, OLED_CMD);  // [5:4] alternative COM pin config

	OLED_WR_Byte(0x81, OLED_CMD);  // Set contrast control
	OLED_WR_Byte(0xEF, OLED_CMD);  // Contrast value (higher = brighter)
	OLED_WR_Byte(0xD9, OLED_CMD);  // Set pre-charge period
	OLED_WR_Byte(0xf1, OLED_CMD);  // [3:0]=Phase1; [7:4]=Phase2
	OLED_WR_Byte(0xDB, OLED_CMD);  // Set VCOMH deselect level
	OLED_WR_Byte(0x30, OLED_CMD);  // 0.83*Vcc

	OLED_WR_Byte(0xA4, OLED_CMD);  // Entire display ON: output follows RAM content
	OLED_WR_Byte(0xA6, OLED_CMD);  // Normal display (not inverted)
	OLED_WR_Byte(0xAF, OLED_CMD);  // Display ON
}

/**************************************************************************
Function: Initialize the SSD1306 OLED controller
Input   : none
Output  : none
**************************************************************************/
void OLED_Init(void)
{
	/* Configure GPIOB: PB3(RST), PB4(SDA), PB5(SCL) as push-pull output */
	RCC->APB2ENR |= 1<<3;
	GPIOB->CRL &= 0XFF000FFF;
	GPIOB->CRL |= 0X00222000;

	/* Configure GPIOA: PA15(DC) as push-pull output */
	RCC->APB2ENR |= 1<<2;
	GPIOA->CRH &= 0X0FFFFFFF;
	GPIOA->CRH |= 0X20000000;

	/* Hardware reset */
	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set();

	OLED_Config();   // SSD1306 initialization command sequence
	OLED_Clear();
}
