#ifndef __BSP_MPU9250_H__
#define __BSP_MPU9250_H__
#include "bsp_common.h"
#include "bsp_mpuiic.h"




#define MPU9250_ADDR            0X68    //MPU9250 device IIC address
#define MPU9250_ID				0X71  	//MPU9250 device ID


//The MPU9250 embeds an AK8963 magnetometer, address and ID as follows:
#define AK8963_ADDR				0X0C	//AK8963 I2C address
#define AK8963_ID				0X48	//AK8963 device ID


//AK8963 internal registers
#define MAG_WIA					0x00	//AK8963 device ID register address
#define MAG_CNTL1          	  	0X0A    
#define MAG_CNTL2            	0X0B

#define MAG_XOUT_L				0X03	
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08

//MPU6500 internal registers
#define MPU_SELF_TESTX_REG		0X0D	//Self-test register X
#define MPU_SELF_TESTY_REG		0X0E	//Self-test register Y
#define MPU_SELF_TESTZ_REG		0X0F	//Self-test register Z
#define MPU_SELF_TESTA_REG		0X10	//Self-test register A
#define MPU_SAMPLE_RATE_REG		0X19	//Sample rate divider
#define MPU_CFG_REG				0X1A	//Configuration register
#define MPU_GYRO_CFG_REG		0X1B	//Gyroscope configuration register
#define MPU_ACCEL_CFG_REG		0X1C	//Accelerometer configuration register
#define MPU_ACCEL_CFG_2_REG     0x1D
#define MPU_MOTION_DET_REG		0X1F	//Motion detection threshold setting register
#define MPU_FIFO_EN_REG			0X23	//FIFO enable register
#define MPU_I2CMST_CTRL_REG		0X24	//IIC master control register
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC slave 0 device address register
#define MPU_I2CSLV0_REG			0X26	//IIC slave 0 data address register
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC slave 0 control register
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC slave 1 device address register
#define MPU_I2CSLV1_REG			0X29	//IIC slave 1 data address register
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC slave 1 control register
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC slave 2 device address register
#define MPU_I2CSLV2_REG			0X2C	//IIC slave 2 data address register
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC slave 2 control register
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC slave 3 device address register
#define MPU_I2CSLV3_REG			0X2F	//IIC slave 3 data address register
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC slave 3 control register
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC slave 4 device address register
#define MPU_I2CSLV4_REG			0X32	//IIC slave 4 data address register
#define MPU_I2CSLV4_DO_REG		0X33	//IIC slave 4 write data register
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC slave 4 control register
#define MPU_I2CSLV4_DI_REG		0X35	//IIC slave 4 read data register

#define MPU_I2CMST_STA_REG		0X36	//IIC master status register
#define MPU_INTBP_CFG_REG		0X37	//Interrupt/bypass setting register
#define MPU_INT_EN_REG			0X38	//Interrupt enable register
#define MPU_INT_STA_REG			0X3A	//Interrupt status register

#define MPU_ACCEL_XOUTH_REG		0X3B	//Acceleration value, X-axis high 8-bit register
#define MPU_ACCEL_XOUTL_REG		0X3C	//Acceleration value, X-axis low 8-bit register
#define MPU_ACCEL_YOUTH_REG		0X3D	//Acceleration value, Y-axis high 8-bit register
#define MPU_ACCEL_YOUTL_REG		0X3E	//Acceleration value, Y-axis low 8-bit register
#define MPU_ACCEL_ZOUTH_REG		0X3F	//Acceleration value, Z-axis high 8-bit register
#define MPU_ACCEL_ZOUTL_REG		0X40	//Acceleration value, Z-axis low 8-bit register

#define MPU_TEMP_OUTH_REG		0X41	//Temperature value high 8-bit register
#define MPU_TEMP_OUTL_REG		0X42	//Temperature value low 8-bit register

#define MPU_GYRO_XOUTH_REG		0X43	//Gyroscope value, X-axis high 8-bit register
#define MPU_GYRO_XOUTL_REG		0X44	//Gyroscope value, X-axis low 8-bit register
#define MPU_GYRO_YOUTH_REG		0X45	//Gyroscope value, Y-axis high 8-bit register
#define MPU_GYRO_YOUTL_REG		0X46	//Gyroscope value, Y-axis low 8-bit register
#define MPU_GYRO_ZOUTH_REG		0X47	//Gyroscope value, Z-axis high 8-bit register
#define MPU_GYRO_ZOUTL_REG		0X48	//Gyroscope value, Z-axis low 8-bit register

#define MPU_EXT_SENS_DATA_00_REG 0x49

#define MPU_I2CSLV0_DO_REG		0X63	//IIC slave 0 data register
#define MPU_I2CSLV1_DO_REG		0X64	//IIC slave 1 data register
#define MPU_I2CSLV2_DO_REG		0X65	//IIC slave 2 data register
#define MPU_I2CSLV3_DO_REG		0X66	//IIC slave 3 data register

#define MPU_I2CMST_DELAY_REG	0X67	//IIC master delay management register
#define MPU_SIGPATH_RST_REG		0X68	//Signal path reset register
#define MPU_MDETECT_CTRL_REG	0X69	//Motion detection control register
#define MPU_USER_CTRL_REG		0X6A	//User control register
#define MPU_PWR_MGMT1_REG		0X6B	//Power management register 1
#define MPU_PWR_MGMT2_REG		0X6C	//Power management register 2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO count register high 8 bits
#define MPU_FIFO_CNTL_REG		0X73	//FIFO count register low 8 bits
#define MPU_FIFO_RW_REG			0X74	//FIFO read/write register
#define MPU_DEVICE_ID_REG		0X75	//Device ID register

typedef struct
{
	short accel[3];
	short gyro[3];
	short compass[3];
    float roll;
    float pitch;
    float yaw;
} mpu_data_t;



uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_Rate(uint16_t rate);


uint8_t MPU9250_Init(void);
void MPU_Init_Ok(void);
void MPU_Delay_ms(uint16_t time);
void MPU9250_Read_Data_Handle(void);
void MPU9250_Send_Raw_Data(void);
void MPU9250_Send_Attitude_Data(void);
void MPU_ADDR_CTRL(void);

uint8_t MPU_Get_Gyroscope(short *gyro);
uint8_t MPU_Get_Accelerometer(short *accel);
uint8_t MPU_Get_Magnetometer(short *mag);


float MPU_Get_Roll_Now(void);
float MPU_Get_Pitch_Now(void);
float MPU_Get_Yaw_Now(void);

#endif
