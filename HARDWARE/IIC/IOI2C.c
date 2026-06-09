#include "ioi2c.h"
#include "sys.h"
#include "delay.h"

/**************************************************************************
Function: Initialize I2C GPIO pins (PB8=SCL, PB9=SDA, open-drain output)
Input   : none
Output  : none
**************************************************************************/
void IIC_Init(void)
{
	RCC->APB2ENR |= 1<<3;         // Enable GPIOB clock
	GPIOB->CRH &= 0XFFFFFF00;    // Clear PB8 and PB9 config
	GPIOB->CRH |= 0X00000033;    // PB8, PB9: open-drain output, 50MHz
}

/**************************************************************************
Function: Generate I2C START condition (SDA falls while SCL is high)
Input   : none
Output  : 1 on success, 0 if bus is busy
**************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();       // Set SDA as output
	IIC_SDA = 1;
	if(!READ_SDA) return 0;   // Bus busy
	IIC_SCL = 1;
	delay_us(1);
	IIC_SDA = 0;     // START: SDA falls while SCL is high
	if(READ_SDA) return 0;
	delay_us(1);
	IIC_SCL = 0;     // Hold bus, ready to send data
	return 1;
}

/**************************************************************************
Function: Generate I2C STOP condition (SDA rises while SCL is high)
Input   : none
Output  : none
**************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();       // Set SDA as output
	IIC_SCL = 0;
	IIC_SDA = 0;     // STOP: SDA rises while SCL is high
	delay_us(1);
	IIC_SCL = 1;
	IIC_SDA = 1;     // Release I2C bus
	delay_us(1);
}

/**************************************************************************
Function: Wait for ACK from slave device
Input   : none
Output  : 1 if ACK received, 0 if timeout (no ACK)
**************************************************************************/
int IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SDA_IN();        // Set SDA as input
	IIC_SDA = 1;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 50)
		{
			IIC_Stop();
			return 0;  // Timeout: no ACK
		}
		delay_us(1);
	}
	IIC_SCL = 0;     // Clock low
	return 1;        // ACK received
}

/**************************************************************************
Function: Send ACK to slave (SDA low during SCL pulse)
Input   : none
Output  : none
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	IIC_SCL = 0;
}

/**************************************************************************
Function: Send NACK to slave (SDA high during SCL pulse)
Input   : none
Output  : none
**************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	IIC_SCL = 0;
}

/**************************************************************************
Function: Send one byte over I2C (MSB first)
Input   : txd - byte to transmit
Output  : none
**************************************************************************/
void IIC_Send_Byte(u8 txd)
{
	u8 t;
	SDA_OUT();
	IIC_SCL = 0;     // Pull clock low to start data transfer
	for(t = 0; t < 8; t++)
	{
		IIC_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		delay_us(1);
		IIC_SCL = 1;
		delay_us(1);
		IIC_SCL = 0;
		delay_us(1);
	}
}

/**************************************************************************
Function: Write multiple bytes to a register of a slave device
Input   : addr - device address, reg - register address,
          len - number of bytes, data - data buffer
Output  : 0 on success, 1 on failure
**************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	if(!IIC_Start())
		return 1;
	IIC_Send_Byte(addr << 1);    // Write address
	if(!IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);          // Register address
	IIC_Wait_Ack();
	for(i = 0; i < len; i++) {
		IIC_Send_Byte(data[i]);
		if(!IIC_Wait_Ack()) {
			IIC_Stop();
			return 0;
		}
	}
	IIC_Stop();
	return 0;
}

/**************************************************************************
Function: Read multiple bytes from a register of a slave device
Input   : addr - device address, reg - register address,
          len - number of bytes, buf - receive buffer
Output  : 0 on success, 1 on failure
**************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(!IIC_Start())
		return 1;
	IIC_Send_Byte(addr << 1);    // Write address (to set register pointer)
	if(!IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);          // Register address
	IIC_Wait_Ack();
	IIC_Start();                 // Repeated START
	IIC_Send_Byte((addr << 1) + 1);  // Read address
	IIC_Wait_Ack();
	while(len) {
		if(len == 1)
			*buf = IIC_Read_Byte(0);  // Last byte: NACK
		else
			*buf = IIC_Read_Byte(1);  // More bytes: ACK
		buf++;
		len--;
	}
	IIC_Stop();
	return 0;
}

/**************************************************************************
Function: Read one byte over I2C (MSB first)
Input   : ack - 1 to send ACK after byte, 0 to send NACK
Output  : received byte
**************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN();    // Set SDA as input
	for(i = 0; i < 8; i++)
	{
		IIC_SCL = 0;
		delay_us(2);
		IIC_SCL = 1;
		receive <<= 1;
		if(READ_SDA) receive++;
		delay_us(2);
	}
	if(ack)
		IIC_Ack();   // Send ACK
	else
		IIC_NAck();  // Send NACK
	return receive;
}

/**************************************************************************
Function: Read one byte from the specified register of a device
Input   : I2C_Addr - device I2C address, addr - register address
Output  : byte read from register
**************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
	unsigned char res = 0;

	IIC_Start();
	IIC_Send_Byte(I2C_Addr);    // Write address
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++; // Register address
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(I2C_Addr + 1); res++;  // Read address
	IIC_Wait_Ack();
	res = IIC_Read_Byte(0);     // Read byte, send NACK
	IIC_Stop();

	return res;
}

/**************************************************************************
Function: Read multiple consecutive bytes from a device register
Input   : dev - device I2C address, reg - register address,
          length - number of bytes, data - receive buffer
Output  : number of bytes read
**************************************************************************/
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
	u8 count = 0;

	IIC_Start();
	IIC_Send_Byte(dev);     // Write address
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);     // Register address
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(dev + 1); // Read address
	IIC_Wait_Ack();

	for(count = 0; count < length; count++)
	{
		if(count != length - 1)
			data[count] = IIC_Read_Byte(1);  // ACK for all but last
		else
			data[count] = IIC_Read_Byte(0);  // NACK for last byte
	}
	IIC_Stop();
	return count;
}

/**************************************************************************
Function: Write multiple bytes to a device register
Input   : dev - device address, reg - register address,
          length - number of bytes, data - data buffer
Output  : 1 (always)
**************************************************************************/
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);      // Write address
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);      // Register address
	IIC_Wait_Ack();
	for(count = 0; count < length; count++)
	{
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	}
	IIC_Stop();
	return 1;
}

/**************************************************************************
Function: Read one byte from a device register
Input   : dev - device address, reg - register address,
          data - pointer to store the result
Output  : 1 (always)
**************************************************************************/
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
	*data = I2C_ReadOneByte(dev, reg);
	return 1;
}

/**************************************************************************
Function: Write one byte to a device register
Input   : dev - device address, reg - register address, data - byte to write
Output  : 1 (always)
**************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
	return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
Function: Read-modify-write multiple bits in a register byte
Input   : dev - device address, reg - register address,
          bitStart - starting bit position, length - number of bits,
          data - new bit values (right-aligned)
Output  : 1 on success, 0 on failure
**************************************************************************/
u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{
	u8 b;
	if(IICreadByte(dev, reg, &b) != 0) {
		u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
		data <<= (8 - length);
		data >>= (7 - bitStart);
		b &= mask;
		b |= data;
		return IICwriteByte(dev, reg, b);
	} else {
		return 0;
	}
}

/**************************************************************************
Function: Read-modify-write a single bit in a register byte
Input   : dev - device address, reg - register address,
          bitNum - bit position to modify,
          data - 0 to clear the bit, non-zero to set it
Output  : 1 on success, 0 on failure
**************************************************************************/
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
	u8 b;
	IICreadByte(dev, reg, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
