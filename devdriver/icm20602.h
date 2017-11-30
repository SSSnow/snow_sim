/*
 * icm20602.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _ICM20602_H_
#define _ICM20602_H_

#include "../mcudriver/spi.h"
#include <stm32f10x_rcc.h>
#include "device.h"
#include "driver.h"

#define ICM20602_CS			   PAout(4)
#define DISABLE_ICM20602       1
#define ENABLE_ICM20602        0
#define DIR_READ               0x80
#define DIR_WRITE			   0x7F
#define M_PI_F			3.14159265358979323846f

// ICM20602 registers
#define ICMREG_WHOAMI			0x75
#define ICMREG_SMPLRT_DIV		0x19
#define ICMREG_CONFIG			0x1A
#define ICMREG_GYRO_CONFIG		0x1B
#define ICMREG_ACCEL_CONFIG		0x1C
#define ICMREG_ACCEL_CONFIG2	0x1D

#define ICMREG_FIFO_EN			0x23
#define ICMREG_INT_PIN_CFG		0x37
#define ICMREG_INT_ENABLE		0x38
#define ICMREG_INT_STATUS		0x3A
#define ICMREG_ACCEL_XOUT_H		0x3B
#define ICMREG_ACCEL_XOUT_L		0x3C
#define ICMREG_ACCEL_YOUT_H		0x3D
#define ICMREG_ACCEL_YOUT_L		0x3E
#define ICMREG_ACCEL_ZOUT_H		0x3F
#define ICMREG_ACCEL_ZOUT_L		0x40
#define ICMREG_TEMP_OUT_H		0x41
#define ICMREG_TEMP_OUT_L		0x42
#define ICMREG_GYRO_XOUT_H		0x43
#define ICMREG_GYRO_XOUT_L		0x44
#define ICMREG_GYRO_YOUT_H		0x45
#define ICMREG_GYRO_YOUT_L		0x46
#define ICMREG_GYRO_ZOUT_H		0x47
#define ICMREG_GYRO_ZOUT_L		0x48
#define ICMREG_USER_CTRL		0x6A
#define ICMREG_PWR_MGMT_1		0x6B
#define ICMREG_PWR_MGMT_2		0x6C
#define ICMREG_FIFO_COUNTH		0x72
#define ICMREG_FIFO_COUNTL		0x73
#define ICMREG_FIFO_R_W			0x74
#define ICMREG_GYRO_SELFTEST_X	0x50
#define ICMREG_GYRO_SELFTEST_Y	0x51
#define ICMREG_GYRO_SELFTEST_Z	0x52
#define ICMREG_ACCEL_SELFTEST_X	0x0D
#define ICMREG_ACCEL_SELFTEST_Y	0x0E
#define ICMREG_ACCEL_SELFTEST_Z	0x0F

#define ICM20602_SELF_TEST_X_GYRO 0X00
#define ICM20602_SELF_TEST_Y_GYRO 0X01
#define ICM20602_SELF_TEST_Z_GYRO 0X02
#define ICM20602_SELF_TEST_X_ACCEL 0X0D
#define ICM20602_SELF_TEST_Y_ACCEL 0X0E
#define ICM20602_SELF_TEST_Z_ACCEL 0X0F

#define ICM20602_XG_OFFS_USRH 0X13
#define ICM20602_XG_OFFS_USRL 0X14
#define ICM20602_YG_OFFS_USRH 0X15
#define ICM20602_YG_OFFS_USRL 0X16
#define ICM20602_ZG_OFFS_USRH 0X17
#define ICM20602_ZG_OFFS_USRL 0X18

#define	ICM20602_ACC_X_H	0x3B
#define	ICM20602_ACC_X_L	0x3C
#define	ICM20602_ACC_Y_H	0x3D
#define	ICM20602_ACC_Y_L	0x3E
#define	ICM20602_ACC_Z_H	0x3F
#define	ICM20602_ACC_Z_L	0x40
#define ICM20602_ACC_OUT_LEN 6

#define	ICM20602_TEMP_H		0x41
#define	ICM20602_TEMP_L		0x42
#define ICM20602_TEMP_OUT_LEN 2

#define	ICM20602_GYRO_X_H	0x43
#define	ICM20602_GYRO_X_L	0x44
#define	ICM20602_GYRO_Y_H	0x45
#define	ICM20602_GYRO_Y_L	0x46
#define	ICM20602_GYRO_Z_H	0x47
#define	ICM20602_GYRO_Z_L	0x48
#define ICM20602_GYRO_OUT_LEN 6

// Configuration bits ICM20602
#define BIT_H_RESET					0x80
#define BITS_INTERNAL_20MHZ			0x00
#define BITS_BESTCLOCK_PLL1			0x01
#define BITS_BESTCLOCK_PLL2			0x02
#define BITS_BESTCLOCK_PLL3			0x03
#define BITS_BESTCLOCK_PLL4			0x04
#define BITS_BESTCLOCK_PLL5			0x05
#define BITS_INTERNAL_20MHZ2		0x06
#define BITS_STOPCLOCK_RESET		0x07

#define ICM20602_ACC_DLPF_CFG_1046HZ_NOLPF	0x00
#define ICM20602_ACC_DLPF_CFG_218HZ		0x01
#define ICM20602_ACC_DLPF_CFG_99HZ		0x02
#define ICM20602_ACC_DLPF_CFG_44HZ		0x03
#define ICM20602_ACC_DLPF_CFG_21HZ		0x04
#define ICM20602_ACC_DLPF_CFG_10HZ		0x05
#define ICM20602_ACC_DLPF_CFG_5HZ		0x06
#define ICM20602_ACC_DLPF_CFG_420HZ		0x07

#define BITS_FS_250DPS				0x00
#define BITS_FS_500DPS				0x08
#define BITS_FS_1000DPS				0x10
#define BITS_FS_2000DPS				0x18

#define ICM20602_ACCEL_SCALE2G 0  //+/- 2g
#define ICM20602_ACCEL_SCALE4G 8  //+/- 4g
#define ICM20602_ACCEL_SCALE8G 16  //+/- 8g
#define ICM20602_ACCEL_SCALE16G 24  //+/- 16g

#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_DATA_RDY_INT_EN			0x00

/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1		0x11
#define MPUREG_ICM_UNDOC1_VALUE	0xc9

#define ICM20602_WHO_AM_I		0x12


#define ICM20602_ACCEL_DEFAULT_RANGE_G			8
#define ICM20602_ACCEL_DEFAULT_RATE				1000
#define ICM20602_ACCEL_MAX_OUTPUT_RATE			280
#define ICM20602_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ICM20602_GYRO_DEFAULT_RANGE_G			8
#define ICM20602_GYRO_DEFAULT_RATE				1000
/* rates need to be the same between accel and gyro */
#define ICM20602_GYRO_MAX_OUTPUT_RATE			ICM20602_ACCEL_MAX_OUTPUT_RATE
#define ICM20602_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define ICM20602_DEFAULT_ONCHIP_FILTER_FREQ		42

//#define ICM20602_ONE_G					9.80665f


typedef struct{

		int16_t X;
		int16_t Y;
		int16_t Z;

}ICM20602_TYPE;

typedef struct
{
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t ax;
	int16_t ay;
	int16_t az;
}T_mpu_20602;

typedef struct{
	uint8_t gx_h;
	uint8_t gx_l;
	uint8_t gy_h;
	uint8_t gy_l;
	uint8_t gz_h;
	uint8_t gz_l;
	uint8_t ax_h;
	uint8_t ax_l;
	uint8_t ay_h;
	uint8_t ay_l;
	uint8_t az_h;
	uint8_t az_l;
}T_20602_reg;

typedef enum
{
	ICM20602_IOCTRL_ACCEL_READ = 0,
	ICM20602_IOCTRL_GYRO_READ = 1,
	ICM20602_IOCTRL_TEMP_READ = 2,
	ICM20602_IOCTRL_ACCEL_M_S2_READ = 3,
	ICM20602_IOCTRL_GYRO_RAD_S_READ = 4,
	ICM20602_IOCTRL_ACC_SCALE_READ = 5,
	ICM20602_IOCTRL_GYRO_SCALE_READ = 6,
	ICM20602_IOCTRL_IMU_READ = 7,
	ICM20602_IOCTRL_REBOOT =8
}ICM20602_IOCTRL;

/*
  The ICM20602 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using SPI at 1MHz.
  For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */

unsigned int icm20602_getID(void);
unsigned int icm20602_register(void);
static void icm20602_cs_config(void);
static void icm20602_spi_init(void);
static uint8_t icm20602_write(uint8_t reg, uint8_t data);
static int icm20602_read(uint8_t reg, uint8_t readLen, uint8_t* readBuffer );
static int icm20602_ioctrl(uint8_t cmd, void* arg);
static int icm20602_init(void);

#endif /* SRC_DEVDRIVER_ICM20602_H_ */
