#include "MPU6050.h"
#include "IOI2C.h"
#include "usart.h"
#include <math.h>

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)     // DMP sample rate: 200 Hz
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f        // 2^30, used to convert DMP quaternion fixed-point to float

short gyro[3], accel[3], sensors;
float Pitch;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* Gyroscope orientation matrix: maps physical sensor axes to robot axes */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                            0,-1, 0,
                                            0, 0, 1};

/* Convert one row of the orientation matrix to a scalar index for DMP */
static unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;
	if      (row[0] > 0) b = 0;
	else if (row[0] < 0) b = 4;
	else if (row[1] > 0) b = 1;
	else if (row[1] < 0) b = 5;
	else if (row[2] > 0) b = 2;
	else if (row[2] < 0) b = 6;
	else                 b = 7;  // error
	return b;
}

/* Convert a 3x3 orientation matrix to a scalar value for DMP orientation API */
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar  = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}

/* Run the MPU6050 self-test and push bias values to DMP if it passes */
static void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	if(result == 0x7) {
		/* All axes passed; apply bias corrections to DMP */
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
	}
}

uint8_t  buffer[14];
int16_t  MPU6050_FIFO[6][11];     // [6 channels][10 samples + average]
int16_t  Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

/**************************************************************************
Function: Push new 6-axis IMU data into a 10-sample sliding window and
          compute the running average (used for simple noise filtering)
Input   : ax,ay,az - accelerometer X,Y,Z raw values
          gx,gy,gz - gyroscope X,Y,Z raw values
Output  : none
**************************************************************************/
void MPU6050_newValues(int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz)
{
	unsigned char i;
	int32_t sum = 0;

	/* Shift existing samples left (oldest discarded) */
	for(i = 1; i < 10; i++) {
		MPU6050_FIFO[0][i-1] = MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1] = MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1] = MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1] = MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1] = MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1] = MPU6050_FIFO[5][i];
	}

	/* Insert newest sample at position [9] */
	MPU6050_FIFO[0][9] = ax;
	MPU6050_FIFO[1][9] = ay;
	MPU6050_FIFO[2][9] = az;
	MPU6050_FIFO[3][9] = gx;
	MPU6050_FIFO[4][9] = gy;
	MPU6050_FIFO[5][9] = gz;

	/* Compute average for each channel and store at index [10] */
	sum = 0;
	for(i = 0; i < 10; i++) sum += MPU6050_FIFO[0][i];
	MPU6050_FIFO[0][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++) sum += MPU6050_FIFO[1][i];
	MPU6050_FIFO[1][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++) sum += MPU6050_FIFO[2][i];
	MPU6050_FIFO[2][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++) sum += MPU6050_FIFO[3][i];
	MPU6050_FIFO[3][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++) sum += MPU6050_FIFO[4][i];
	MPU6050_FIFO[4][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++) sum += MPU6050_FIFO[5][i];
	MPU6050_FIFO[5][10] = sum / 10;
}

/**************************************************************************
Function: Set the MPU6050 clock source
Input   : source - clock source selection
          CLK_SEL | Clock Source
          --------+--------------------------------------
          0       | Internal oscillator
          1       | PLL with X Gyro reference
          2       | PLL with Y Gyro reference
          3       | PLL with Z Gyro reference
          4       | PLL with external 32.768kHz reference
          5       | PLL with external 19.2MHz reference
          6       | Reserved
          7       | Stops the clock and keeps the timing generator in reset
Output  : none
**************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1,
	             MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/**************************************************************************
Function: Set the full-scale gyroscope measurement range
Input   : range - MPU6050_GYRO_FS_250/500/1000/2000
Output  : none
**************************************************************************/
void MPU6050_setFullScaleGyroRange(uint8_t range)
{
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG,
	             MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set the full-scale accelerometer measurement range
Input   : range - MPU6050_ACCEL_FS_2/4/8/16
Output  : none
**************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range)
{
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG,
	             MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Enable or disable MPU6050 sleep mode
Input   : enabled - 1=enter sleep, 0=wake up
Output  : none
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read the WHO_AM_I register to verify device identity
Input   : none
Output  : 0x68 if the correct device is connected
**************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
	IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
	return buffer[0];
}

/**************************************************************************
Function: Test whether the MPU6050 is responding on the I2C bus
Input   : none
Output  : 1 if connected and responding, 0 otherwise
**************************************************************************/
uint8_t MPU6050_testConnection(void)
{
	if(MPU6050_getDeviceID() == 0x68)
		return 1;
	else
		return 0;
}

/**************************************************************************
Function: Enable or disable MPU6050 as I2C master of the auxiliary bus
Input   : enabled - 1=master mode on, 0=off
Output  : none
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL,
	            MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Enable or disable I2C bypass so the host can access auxiliary I2C
Input   : enabled - 1=bypass on, 0=bypass off
Output  : none
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG,
	            MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: Initialize MPU6050: set clock, gyro range, accel range, wake up,
          and disable auxiliary I2C master/bypass
Input   : none
Output  : none
**************************************************************************/
void MPU6050_initialize(void)
{
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO);      // PLL with Y gyro reference
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);  // Gyro: +/- 2000 deg/s
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // Accel: +/- 2g
	MPU6050_setSleepEnabled(0);                            // Wake up (exit sleep mode)
	MPU6050_setI2CMasterModeEnabled(0);                   // Disable aux I2C master
	MPU6050_setI2CBypassEnabled(0);                       // Disable I2C bypass
}

/**************************************************************************
Function: Initialize the DMP (Digital Motion Processor) inside MPU6050.
          Loads firmware, configures orientation, enables features, runs
          self-test, and starts the DMP. Resets the MCU if WHO_AM_I fails.
Input   : none
Output  : none
**************************************************************************/
void DMP_Init(void)
{
	u8 temp[1] = {0};
	i2cRead(0x68, 0x75, 1, temp);
	if(temp[0] != 0x68) NVIC_SystemReset();  // Wrong device; reset MCU

	if(!mpu_init())
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {}
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {}
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ)) {}
		if(!dmp_load_motion_driver_firmware()) {}
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {}
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		      DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
		      DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {}
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {}
		run_self_test();
		if(!mpu_set_dmp_state(1)) {}
	}
}

/**************************************************************************
Function: Read latest attitude data from the DMP FIFO and compute pitch angle
Input   : none
Output  : none (updates global Pitch variable)
**************************************************************************/
void Read_DMP(void)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if(sensors & INV_WXYZ_QUAT)
	{
		/* Convert fixed-point quaternion (Q30) to float */
		q0 = quat[0] / q30;
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		/* Balance axis = forward/backward tilt = rotation about Y (true pitch).
		 * (The old formula was rotation about X / roll -> only responded to
		 *  left-right tilt.) Pair this with gyro[1] in Get_Angle(). */
		Pitch = asin(2*(q0*q2 - q1*q3)) * 57.3;
	}
}

/**************************************************************************
Function: Break the boot deadlock. The DMP INT is latched (BIT_LATCH_EN |
          BIT_ANY_RD_CLR). If the FIFO backs up/overflows before the EXTI ISR
          starts draining it, the latched INT stays low, no fresh falling edge
          is ever produced, and the ISR never fires. Reset the FIFO and read
          INT_STATUS (any-read clears the latch) so the line releases and the
          next DMP sample makes a clean falling edge that starts the IRQ chain.
          Call ONCE right after EXTI_Init().
**************************************************************************/
void MPU_ClearINT(void)
{
	u8 tmp[1] = {0};
	i2cRead(0x68, MPU6050_RA_INT_STATUS,     1, tmp);  // 0x3A: release the latched data-ready INT
	i2cRead(0x68, MPU6050_RA_DMP_INT_STATUS, 1, tmp);  // 0x39: release the latched DMP INT
}

//------------------End of File----------------------------
