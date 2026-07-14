#include "bsp_mpu9250.h"
#include "bsp.h"
#include "app.h"
#include "app_math.h"
#include "bsp_mpuiic.h"


#include "app_angle.h"
#include "app_motion.h"

#define ENABLE_MPU9250_FILTER   1
#define CONTROL_DELAY           1000


uint8_t g_mpu_init = 0;
uint8_t g_mpu_start = 0;
uint8_t g_mpu_test = 0;


//Zero drift count
int Deviation_Count = 0;
//Gyroscope static offset, raw data
short Deviation_gyro[3], Original_gyro[3];    

float pitch, roll, yaw;	       //Euler angles
mpu_data_t mpu_data = {0};


// Set the IIC address of the MPU9250 to 0x68
void MPU_ADDR_CTRL(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	/*Set the GPIO mode to general-purpose push-pull output*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}


static uint8_t MPU_Basic_Init(void)
{
	uint8_t res = 0;
	MPU_ADDR_CTRL();
	MPU_IIC_Init();
	MPU_Delay_ms(10);

	MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X80); // Reset MPU9250 //Reset MPU9250
	MPU_Delay_ms(100);										   // Delay 100 ms //Delay 100ms
	MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X00); // Wake mpu9250 //Wake up MPU9250

	MPU_Set_Gyro_Fsr(1);  // Gyroscope sensor              //Gyroscope sensor,±500dps=±500°/s ±32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
	MPU_Set_Accel_Fsr(0); // Acceleration sensor           //Acceleration sensor,±2g=±2*9.8m/s^2 ±32768 accel/32768*19.6=accel/1671.84
	MPU_Set_Rate(50);	  // Set the sampling rate to 50Hz //Set the sampling rate to 50Hz

	MPU_Write_Byte(MPU9250_ADDR, MPU_INT_EN_REG, 0X00);	   // Turn off all interrupts //Turn off all interrupts
	MPU_Write_Byte(MPU9250_ADDR, MPU_USER_CTRL_REG, 0X00); // The I2C main mode is off //I2C master mode off
	MPU_Write_Byte(MPU9250_ADDR, MPU_FIFO_EN_REG, 0X00);   // Close the FIFO //Close the FIFO
	// The INT pin is low, enabling bypass mode to read the magnetometer directly
	// The INT pin is active low, enable bypass mode, the magnetometer can be read directly
	MPU_Write_Byte(MPU9250_ADDR, MPU_INTBP_CFG_REG, 0X82);
	// Read the ID of MPU9250 Read the ID of the MPU9250
	res = MPU_Read_Byte(MPU9250_ADDR, MPU_DEVICE_ID_REG);
	printf("MPU9250 RD ID=0x%02X\n", res);
	if (res == MPU9250_ID) // The device ID is correct //Device ID is correct
	{
		MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X01); // Set CLKSEL,PLL X axis as reference //Set CLKSEL, PLL X axis as reference
		MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT2_REG, 0X00); // Acceleration and gyroscope both work //Both accelerometer and gyroscope work
		MPU_Set_Rate(50);									   // Set the sampling rate to 50Hz //Set the sampling rate to 50Hz
	}
	else
		return 1;

	res = MPU_Read_Byte(AK8963_ADDR, MAG_WIA); // Read AK8963ID //Read AK8963 ID
	printf("AK8963 RD ID=0x%02X\n", res);
	if (res == AK8963_ID)
	{
		MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11); // Set AK8963 to single measurement mode //Set AK8963 to single measurement mode
	}
	else
		return 2;
	MPU_Delay_ms(30);
	return 0;
}



//Initialize MPU9250
uint8_t MPU9250_Init(void)
{
	uint8_t res = 0;
	uint8_t count = 0;
	while (1)
	{
		res = MPU_Basic_Init();
		if (res == 0)
		{
			printf("MPU9250 Basic Init OK\n");
			while (Deviation_Count < CONTROL_DELAY)
			{
				MPU_Get_Gyroscope(mpu_data.gyro);
				MPU_Delay_ms(10);
			}
			break;
		}
		else
		{
			count++;
			printf("MPU9250 Basic Init Error:%d\n", res);
			if (count > 5)
			{
				return 1;
			}
		}
		MPU_Delay_ms(200);
	}
	return 0;
}

void MPU_Init_Ok(void)
{
	g_mpu_init = 1;
}

// Set the full-scale range of the MPU9250 gyroscope sensor
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// Return value:0,setting succeeded, other,setting failed  
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, fsr << 3); //Set the gyroscope full-scale range
}

// Set the full-scale range of the MPU9250 acceleration sensor
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// Return value:0,setting succeeded, other,setting failed
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, fsr << 3); //Set the acceleration sensor full-scale range
}

//Set the digital low-pass filter of the MPU9250
// lpf:digital low-pass filter frequency(Hz)
// Return value:0,setting succeeded, other,setting failed
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU9250_ADDR, MPU_CFG_REG, data); //Set the digital low-pass filter
}

//Set the sampling rate of the MPU9250 (assuming Fs=1KHz)
// rate:4~1000(Hz)
// Return value:0,setting succeeded, other,setting failed
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, data); //Set the digital low-pass filter
	return MPU_Set_LPF(rate / 2);									//Automatically set LPF to half the sampling rate
}

//Get the gyroscope values (raw values)
// gx,gy,gz:raw readings of the gyroscope x,y,z axes (signed)
// Return value:0,setting succeeded, other,setting failed
uint8_t MPU_Get_Gyroscope(short *gyro)
{
	#if ENABLE_MPU9250_FILTER
	uint8_t buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		if(Deviation_Count<CONTROL_DELAY)
		{
			Deviation_Count++;
			//Read the gyroscope zero //Read the gyroscope zero point
			Deviation_gyro[0] = (((uint16_t)buf[0]<<8)|buf[1]);  
			Deviation_gyro[1] = (((uint16_t)buf[2]<<8)|buf[3]);  
			Deviation_gyro[2] = (((uint16_t)buf[4]<<8)|buf[5]);	
		}
		else
		{
			// if (g_mpu_start == 0)
			// {
			// 	g_mpu_start = 1;
			// 	Beep_On_Time(100);
			// 	DEBUG("OFFSET GYRO:%d, %d, %d", Deviation_gyro[0], Deviation_gyro[1], Deviation_gyro[2]);
			// }
			//Save the raw data
			//Save the raw data
			Original_gyro[0] = (((uint16_t)buf[0]<<8)|buf[1]);
			Original_gyro[1] = (((uint16_t)buf[2]<<8)|buf[3]);
			Original_gyro[2] = (((uint16_t)buf[4]<<8)|buf[5]);
			
			//Removes zero drift data
			//Remove the zero drift from the data
			gyro[0] = Original_gyro[0]-Deviation_gyro[0];
			gyro[1] = Original_gyro[1]-Deviation_gyro[1];
			gyro[2] = Original_gyro[2]-Deviation_gyro[2];
		}
	}
	return 0;
	#else
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		gyro[0] = ((uint16_t)buf[0] << 8) | buf[1];
		gyro[1] = ((uint16_t)buf[2] << 8) | buf[3];
		gyro[2] = ((uint16_t)buf[4] << 8) | buf[5];
	}
	g_mpu_start = 1;
	return res;
	#endif
}

//Get the acceleration values (raw values)
// gx,gy,gz:raw readings of the gyroscope x,y,z axes (signed)
// Return value:0,setting succeeded, other,setting failed
uint8_t MPU_Get_Accelerometer(short *accel)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		accel[0] =((uint16_t)buf[0]<<8)|buf[1];
		accel[1] =((uint16_t)buf[2]<<8)|buf[3];
		accel[2] =((uint16_t)buf[4]<<8)|buf[5];
	}
	return res;
}

//Get the magnetometer values (raw values)
// mx,my,mz:raw readings of the magnetometer x,y,z axes (signed)
// Return value:0,setting succeeded, other,setting failed
uint8_t MPU_Get_Magnetometer(short *mag)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(AK8963_ADDR, MAG_XOUT_L, 6, buf);
	if (res == 0)
	{
		mag[0] = ((uint16_t)buf[1]<<8)|buf[0];
		mag[1] = ((uint16_t)buf[3]<<8)|buf[2];
		mag[2] = ((uint16_t)buf[5]<<8)|buf[4];
	}
	MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11); // The AK8963 must be set back to single measurement mode after every read
	return res;
}


void MPU_Delay_ms(uint16_t time)
{
	if (App_FreeRTOS_Enable() == 0)
	{
		delay_ms(time);
	}
	else
	{
		App_Delay_ms(time);
	}
}


// Get the current roll angle
float MPU_Get_Roll_Now(void)
{
    return mpu_data.roll;         // Return radians
    // return mpu_data.roll*RtA;  // Return degrees
}

// Get the current pitch angle
float MPU_Get_Pitch_Now(void)
{
    return mpu_data.pitch;         // Return radians
    // return mpu_data.pitch*RtA;  // Return degrees
}

// Get the current yaw angle
float MPU_Get_Yaw_Now(void)
{
	return mpu_data.yaw;           // Return radians
	// return mpu_data.yaw*RtA;    // Return degrees
}



// MPU9250 data reading thread
void MPU9250_Read_Data_Handle(void)
{
	if (Deviation_Count >= CONTROL_DELAY)
	{
		MPU_Get_Gyroscope(mpu_data.gyro);
		MPU_Get_Accelerometer(mpu_data.accel);
		MPU_Get_Magnetometer(mpu_data.compass);
		
		g_imu_data.accX = mpu_data.accel[0];
		g_imu_data.accY = mpu_data.accel[1];
		g_imu_data.accZ = mpu_data.accel[2];
		g_imu_data.gyroX = mpu_data.gyro[0];
		g_imu_data.gyroY = mpu_data.gyro[1];
		g_imu_data.gyroZ = mpu_data.gyro[2];
		// Attitude solving to obtain the attitude angles
		get_attitude_angle(&g_imu_data, &g_attitude, DT);
		#if ENABLE_ROLL_PITCH
		if(Motion_Get_Car_Type() == CAR_SUNRISE)
		{
			// The Sunrise Pi car is mounted vertically, so special handling is required
			mpu_data.roll = g_attitude.roll>(M_PI*0.5)?g_attitude.roll+(M_PI*0.5)-2*M_PI:g_attitude.roll+(M_PI*0.5);
			mpu_data.pitch = g_attitude.pitch;
		}
		else
		{
			mpu_data.roll = g_attitude.roll;
			mpu_data.pitch = g_attitude.pitch;
		}
		#endif
		mpu_data.yaw = g_attitude.yaw;
		#if ENABLE_DEBUG_MPU_ATT
		printf("roll:%f, pitch:%f, yaw:%f\n", mpu_data.roll*57.3, mpu_data.pitch*57.3, mpu_data.yaw*57.3);
		#endif
	}
	else
	{
		MPU_Get_Gyroscope(mpu_data.gyro);
		MPU_Get_Accelerometer(mpu_data.accel);
		MPU_Get_Magnetometer(mpu_data.compass);
	}
}

void MPU9250_Send_Raw_Data(void)
{
#define LEN 23
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID - 1;
	data_buffer[2] = LEN - 2;			  // Count
	data_buffer[3] = FUNC_REPORT_MPU_RAW; // Function byte
	data_buffer[4] = mpu_data.gyro[0] & 0xff;
	data_buffer[5] = (mpu_data.gyro[0] >> 8) & 0xff;
	data_buffer[6] = mpu_data.gyro[1] & 0xff;
	data_buffer[7] = (mpu_data.gyro[1] >> 8) & 0xff;
	data_buffer[8] = mpu_data.gyro[2] & 0xff;
	data_buffer[9] = (mpu_data.gyro[2] >> 8) & 0xff;

	data_buffer[10] = mpu_data.accel[0] & 0xff;
	data_buffer[11] = (mpu_data.accel[0] >> 8) & 0xff;
	data_buffer[12] = mpu_data.accel[1] & 0xff;
	data_buffer[13] = (mpu_data.accel[1] >> 8) & 0xff;
	data_buffer[14] = mpu_data.accel[2] & 0xff;
	data_buffer[15] = (mpu_data.accel[2] >> 8) & 0xff;

	data_buffer[16] = mpu_data.compass[0] & 0xff;
	data_buffer[17] = (mpu_data.compass[0] >> 8) & 0xff;
	data_buffer[18] = mpu_data.compass[1] & 0xff;
	data_buffer[19] = (mpu_data.compass[1] >> 8) & 0xff;
	data_buffer[20] = mpu_data.compass[2] & 0xff;
	data_buffer[21] = (mpu_data.compass[2] >> 8) & 0xff;

	for (i = 2; i < LEN - 1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN - 1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}


// Send the attitude angle data to the main controller, unit: radians
void MPU9250_Send_Attitude_Data(void)
{
    #define LENS        11
	uint8_t data_buffer[LENS] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LENS-2; // Count
	data_buffer[3] = FUNC_REPORT_IMU_ATT;    // Function byte
	data_buffer[4] = (int)(MPU_Get_Roll_Now()*10000) & 0xff;
	data_buffer[5] = ((int)(MPU_Get_Roll_Now()*10000) >> 8) & 0xff;
	data_buffer[6] = (int)(MPU_Get_Pitch_Now()*10000) & 0xff;
	data_buffer[7] = ((int)(MPU_Get_Pitch_Now()*10000) >> 8) & 0xff;
	data_buffer[8] = (int)(MPU_Get_Yaw_Now()*10000) & 0xff;
	data_buffer[9] = ((int)(MPU_Get_Yaw_Now()*10000) >> 8) & 0xff;

	for (i = 2; i < LENS-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LENS-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}

