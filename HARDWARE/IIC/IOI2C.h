#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f10x.h"

/* Bit-band access macros for single-bit GPIO manipulation */
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)        *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

/* GPIO output data register addresses */
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C

/* GPIO input data register addresses */
#define GPIOA_IDR_Addr    (GPIOA_BASE+8)  //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8)  //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8)  //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8)  //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8)  //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8)  //0x40011E08

/* Single-bit GPIO macros are defined in sys.h, not redefined here */

/* Switch SDA (PB9) between input and output modes */
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

/* I2C pin definitions: SCL=PB8, SDA=PB9 */
#define IIC_SCL    PBout(8)  // SCL output
#define IIC_SDA    PBout(9)  // SDA output
#define READ_SDA   PBin(9)   // SDA input (read)

/* I2C software driver API */
void IIC_Init(void);                          // Initialize I2C GPIO pins
int  IIC_Start(void);                         // Generate I2C START condition
void IIC_Stop(void);                          // Generate I2C STOP condition
void IIC_Send_Byte(u8 txd);                   // Send one byte
u8   IIC_Read_Byte(unsigned char ack);        // Read one byte, send ACK or NACK
int  IIC_Wait_Ack(void);                      // Wait for ACK from slave
void IIC_Ack(void);                           // Send ACK
void IIC_NAck(void);                          // Send NACK

void IIC_Write_One_Byte(u8 daddr, u8 addr, u8 data);
u8   IIC_Read_One_Byte(u8 daddr, u8 addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8   IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8   IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data);
u8   IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data);
u8   IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int  i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int  i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

//------------------End of File----------------------------
