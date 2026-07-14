#include "protocol.h"
#include "config.h"
#include "yb_debug.h"
#include "app.h"


#include "bsp.h"
#include "app_motion.h"
#include "app_uart_servo.h"
#include "app_pid.h"
#include "app_flash.h"
#include "app_rgb.h"
#include "app_oled.h"
#include "app_ackerman.h"



/* Command receive buffer */
uint8_t RxBuffer[PTO_MAX_BUF_LEN];
/* Receive data index */
uint8_t RxIndex = 0;
/* Receive state machine */
uint8_t RxFlag = 0;
/* New command received flag */
uint8_t New_CMD_flag;
/* New command data length */
uint8_t New_CMD_length;

/* Request data flag */
uint8_t g_Request_Flag = 0;
uint8_t g_Request_ARM_ID = 0;
uint8_t g_Request_Parm = 0;


// Request data handler function
void Request_Data(uint8_t request, uint8_t parm)
{
	g_Request_Flag = request;
	g_Request_ARM_ID = parm;
	g_Request_Parm = parm;
	if (g_Request_Flag == FUNC_UART_SERVO && g_Request_ARM_ID > 0)
	{
		UartServo_Clear_Flag(FLAG_RXBUFF);
		UartServo_Set_Read_State(ARM_READ_TO_UART);
		UartServo_Get_Angle(g_Request_ARM_ID);
	}
	else if(g_Request_Flag == FUNC_ARM_OFFSET)
	{
		if (g_Request_ARM_ID != 0)
		{
			UartServo_Clear_Flag(FLAG_RXBUFF);
			UartServo_Set_Read_State(ARM_READ_TO_FLASH);
			UartServo_Get_Angle(g_Request_ARM_ID);
		}
	}
	else if (g_Request_Flag == FUNC_ARM_CTRL)
	{
		g_Request_ARM_ID = 1;
		UartServo_Clear_Flag(FLAG_RXBUFF);
		UartServo_Clear_Arm_Read_Vlaue();
		UartServo_Set_Read_State(ARM_READ_TO_ARM);
		UartServo_Get_Angle(g_Request_ARM_ID);
	}
}

// Send the requested data
void Send_Request_Data(void)
{
	static uint16_t request_count = 0;
	switch (g_Request_Flag)
	{
	/* Read the current version number */
	case FUNC_VERSION:
	{
		App_Send_Version();
		g_Request_Flag = 0;
		break;
	}
	/* Read the car motion information */
	case FUNC_AUTO_REPORT:
	{
		Motion_Send_Data();
		g_Request_Flag = 0;
		break;
	}
	/* Read the bus servo position */
	case FUNC_UART_SERVO:
	{
		if (UartServo_Rx_Parse(g_Request_ARM_ID))
		{
			g_Request_Flag = 0;
		}
		break;
	}
	/* Read the positions of the six joints of the robotic arm */
	case FUNC_ARM_CTRL:
	{
		static uint8_t next_servo = 0;
		if (UartServo_Rx_Parse(g_Request_ARM_ID))
		{
			next_servo = 1;
		}
		else
		{
			if (request_count >= 3)
			{
				next_servo = 1;
			}
		}
		if (next_servo)
		{
			next_servo = 0;
			request_count = 0;
			g_Request_ARM_ID++;
			if (g_Request_ARM_ID >= 7) // End condition, finish regardless of whether data was received.
			{
				g_Request_ARM_ID = 0;
				g_Request_Flag = 0;
				UartServo_Send_ARM_Angle_Array();
				DEBUG("Read ARM Array END\n");
			}
			else
			{
				UartServo_Get_Angle(g_Request_ARM_ID);
			}
		}
		break;
	}
	/* Set the robotic arm median offset */
	case FUNC_ARM_OFFSET:
	{
		if(g_Request_ARM_ID == 0)
		{
			g_Request_Flag = 0;
			UartServo_Send_Offset_State(0, FLAG_OFFSET_OK);
		}
		if (UartServo_Rx_Parse(g_Request_ARM_ID))
		{
			g_Request_Flag = 0;
		}
		break;
	}
	/* Read the PID parameters */
	case FUNC_SET_MOTOR_PID:
	{
		if (g_Request_Parm > 0 && g_Request_Parm < 5)
		{
			PID_Send_Parm_Active(g_Request_Parm);
		}
		g_Request_Flag = 0;
		break;
	}

	/* Read the yaw PID parameters */
	case FUNC_SET_YAW_PID:
	{
		if (g_Request_Parm == 5)
		{
			PID_Send_Parm_Active(5);
		}
		g_Request_Flag = 0;
		break;
	}

	case FUNC_AKM_DEF_ANGLE:
	{
		Ackerman_Send_Default_Angle();
		g_Request_Flag = 0;
		break;
	}

	case FUNC_CAR_TYPE:
	{
		Motion_Send_Car_Type();
		g_Request_Flag = 0;
		break;
	}

	/* Other values, clear the state */
	default:
		g_Request_Flag = 0;
		break;
	}

	if (g_Request_Flag == 0)
	{
		request_count = 0;
	}
	else // A data request is pending, waiting to send, exits after a 5-count timeout
	{
		request_count++;
		if (request_count >= 5)
		{
			g_Request_Flag = 0;
			request_count = 0;
			if (UartServo_Get_Read_State() == ARM_READ_TO_FLASH)
			{
				UartServo_Send_Offset_State(g_Request_ARM_ID, FLAG_OFFSET_ERROR);
			}
		}
	}
}

// Get the request data flag
uint8_t Get_Request_Flag(void)
{
	return g_Request_Flag;
}

// Get the received data
uint8_t* Get_RxBuffer(void)
{
	return (uint8_t*)RxBuffer;
}

// Get the command length
uint8_t Get_CMD_Length(void)
{
	return New_CMD_length;
}

// Get the command flag
uint8_t Get_CMD_Flag(void)
{
	return New_CMD_flag;
}

// Clear the command data and related flags
void Clear_CMD_Flag(void)
{
	#if ENABLE_CLEAR_RXBUF
	for (uint8_t i = 0; i < New_CMD_length; i++)
	{
		RxBuffer[i] = 0;
	}
	#endif
	New_CMD_length = 0;
	New_CMD_flag = 0;
}

//Clear all values in RxBuffer to 0
void Clear_RxBuffer(void)
{
	for (uint8_t i = 0; i < PTO_MAX_BUF_LEN; i++)
	{
		RxBuffer[i] = 0;
	}
}

// Content that can still be parsed when the battery is low
void Upper_Data_Parse_Low_Battery(uint8_t *data_buf, uint8_t num)
{
	#if ENABLE_CHECKSUM
	// Head1 Head2 Length Function Param x Checksum
	/* First compute the checksum accumulation */
	int sum = 0;
	for (uint8_t i = 2; i < (num - 1); i++)
		sum += *(data_buf + i);
	sum = sum & 0xFF;
	/* Check the accumulated checksum, discard the frame if it differs*/
	uint8_t recvSum = *(data_buf + num - 1);
	if (!(sum == recvSum))
	{
		DEBUG("Check sum error!, CalSum:%d, recvSum:%d\n", sum, recvSum);
		return;
	}
	#endif

	uint8_t func_id = *(data_buf + 3);
	// DEBUG("func_id=%d ", func_id);
	switch (func_id)
	{
	/* Check function byte: reset the car state, including stopping, lights off, buzzer off */
	case FUNC_RESET_STATE:
	{
		u8 verify = *(data_buf + 4);
		if (verify == SAVE_VERIFY)
		{
			Motion_Ctrl_State(MOTION_BRAKE, 0, 0);
			app_rgb_set_effect(RGB_NO_EFFECT, 5);
			RGB_Clear();
			RGB_Update();
			Beep_On_Time(0);
		}
		break;
	}

	/* Check function byte: set the car type */
	case FUNC_CAR_TYPE:
	{
		uint8_t carType = *(data_buf + 4);
		uint8_t verify = *(data_buf + 5);
		DEBUG("SET_CAR_TYPE=0x%02X, 0x%02X", carType, verify);
		if (verify == SAVE_VERIFY)
		{
			if (carType < CAR_TYPE_MAX)
			{
				Motion_Set_Car_Type((car_type_t)carType);
				Flash_Set_CarType(carType);
				if (Motion_Get_Car_Type() ==  CAR_SUNRISE)
				{
					PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
					Flash_Set_PID(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
				}
			}
		}
		else
		{
			if (carType < CAR_TYPE_MAX)
			{
				Motion_Set_Car_Type((car_type_t)carType);
				if (Motion_Get_Car_Type() ==  CAR_SUNRISE)
				{
					PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
				}
			}
		}
		App_Delay_ms(50);
		Bsp_Reset_MCU();
		break;
	}

	/* Request data */
	case FUNC_REQUEST_DATA:
	{
		uint8_t request = *(data_buf + 4);
		uint8_t param = *(data_buf + 5);
		DEBUG("request=0x%02X, %d\n", request, param);
		Request_Data(request, param);
		break;
	}

	default:
		break;
	}
}

/**
 * @Brief: Data parsing
 * @Note: 
 * @Parm: Pass in a received data frame and its length
 * @Retval: 
 */
void Upper_Data_Parse(uint8_t *data_buf, uint8_t num)
{
	#if ENABLE_CHECKSUM
	// Head1 Head2 Length Function Param x Checksum
	/* First compute the checksum accumulation */
	int sum = 0;
	for (uint8_t i = 2; i < (num - 1); i++)
		sum += *(data_buf + i);
	sum = sum & 0xFF;
	/* Check the accumulated checksum, discard the frame if it differs*/
	uint8_t recvSum = *(data_buf + num - 1);
	if (!(sum == recvSum))
	{
		DEBUG("Check sum error!, CalSum:%d, recvSum:%d\n", sum, recvSum);
		return;
	}
	#endif

	/* Check the frame header ---- already checked while receiving, no need for redundant work */
	// if (!(*(data_buf) == PTO_HEAD && *(data_buf + 1) == PTO_DEVICE_ID))
	// {
	// 	DEBUG("Check Head & id error!\n");
	// 	return;
	// }

	uint8_t func_id = *(data_buf + 3);
	// DEBUG("func_id=%d ", func_id);
	switch (func_id)
	{
	// /* Auto-upload low-level data */
	case FUNC_AUTO_REPORT:
	{
		uint8_t state = *(data_buf + 4);
		uint8_t forever = *(data_buf + 5);
		DEBUG("report:%d, %d\n", state, forever);
		Set_Auto_Report(state);
		// Check whether to permanently save it to flash
		if (forever == SAVE_VERIFY)
		{
			Flash_Set_Auto_Report(state);
		}
		break;
	}

	/* Check function byte: buzzer control */
	case FUNC_BEEP:
	{
		uint16_t time = *(data_buf + 5) << 8 | *(data_buf + 4);
		DEBUG("beep:%d\n", time);
		Beep_On_Time(time);
		break;
	}

	/* Check function byte: control a single servo */
	case FUNC_PWM_SERVO:
	{
		uint8_t servo_id = *(data_buf + 4);
		uint8_t angle = *(data_buf + 5);
		DEBUG("pwmServo:%d, %d\n", servo_id, angle);
		PwmServo_Set_Angle(servo_id - 1, angle);
		break;
	}

	/* Check function byte: control all servos */
	case FUNC_PWM_SERVO_ALL:
	{
		uint8_t angle_s1 = *(data_buf + 4);
		uint8_t angle_s2 = *(data_buf + 5);
		uint8_t angle_s3 = *(data_buf + 6);
		uint8_t angle_s4 = *(data_buf + 7);
		DEBUG("all Servo:%d, %d, %d, %d\n", angle_s1, angle_s2, angle_s3, angle_s4);
		PwmServo_Set_Angle_All(angle_s1, angle_s2, angle_s3, angle_s4);
		break;
	}

	/* Check function byte: RGB light control */
	case FUNC_RGB:
	{
		u8 index = *(data_buf + 4);
		u8 red = *(data_buf + 5);
		u8 green = *(data_buf + 6);
		u8 blue = *(data_buf + 7);
		RGB_Set_Color(index, red, green, blue);
		DEBUG("RGB:%d, %d, %d, %d\n", index, red, green, blue);
		RGB_Update();
		break;
	}

	/* Check function byte: RGB light effect control */
	case FUNC_RGB_EFFECT:
	{
		u8 effect = *(data_buf + 4);
		u8 speed = *(data_buf + 5);
		if (app_rgb_get_effect() != effect || effect == 0)
		{
			RGB_Clear();
			RGB_Update();
		}
		app_rgb_set_effect(effect, speed);
		if (effect == RGB_BREATHING)
		{
			u8 breath_color = *(data_buf + 6);
			app_rgb_set_breathing_color(breath_color);
		}
		break;
	}

	/* Check function byte: reset the car state, including stopping, lights off, buzzer off */
	case FUNC_RESET_STATE:
	{
		u8 verify = *(data_buf + 4);
		if (verify == SAVE_VERIFY)
		{
			Motion_Ctrl_State(MOTION_BRAKE, 0, 0);
			app_rgb_set_effect(RGB_NO_EFFECT, 5);
			RGB_Clear();
			RGB_Update();
			Beep_On_Time(0);
		}
		break;
	}

	/* Control the motors, encoder not used */
	case FUNC_MOTOR:
	{
		int16_t speed[4] = {0};
		int8_t motor_1 = *(data_buf + 4);
		int8_t motor_2 = *(data_buf + 5);
		int8_t motor_3 = *(data_buf + 6);
		int8_t motor_4 = *(data_buf + 7);
		// uint8_t pid = *(data_buf + 8);
		DEBUG("motor=%d, %d, %d, %d", motor_1, motor_2, motor_3, motor_4);
		
		int16_t motor_pulse = MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE;
		if (Motion_Get_Car_Type() == CAR_SUNRISE)
		{
			motor_pulse = MOTOR_MAX_PULSE - MOTOR_SUNRISE_IGNORE_PULSE;
		}
		speed[0] = (int16_t)motor_1 * (motor_pulse / 100.0);
		speed[1] = (int16_t)motor_2 * (motor_pulse / 100.0);
		speed[2] = (int16_t)motor_3 * (motor_pulse / 100.0);
		speed[3] = (int16_t)motor_4 * (motor_pulse / 100.0);
		// PWM control of the car motion
		Motion_Set_Pwm(speed[0], speed[1], speed[2], speed[3]);

		// speed[0] = (int16_t)motor_1 * (10);
		// speed[1] = (int16_t)motor_2 * (10);
		// speed[2] = (int16_t)motor_3 * (10);
		// speed[3] = (int16_t)motor_4 * (10);
		// // Control the car motion, Motor_X=[-1000, 1000].
		// Motion_Set_Speed(speed[0], speed[1], speed[2], speed[3]);
		break;
	}

	/* Control the car motion */
	case FUNC_CAR_RUN:
	{
		uint8_t parm = *(data_buf + 4);
		uint8_t state = *(data_buf + 5);
		uint16_t speed = *(data_buf + 7) << 8 | *(data_buf + 6);
		DEBUG("car_run=0x%02X, %d, %d", parm, state, speed);
		uint8_t adjust = parm & 0x80;
		Motion_Ctrl_State(state, speed, (adjust==0?0:1));
		break;
	}

	/* Check function byte: car speed setting */
	case FUNC_MOTION:
	{
		uint8_t parm = (uint8_t) *(data_buf + 4);
		int16_t Vx_recv = *(data_buf + 6) << 8 | *(data_buf + 5);
		int16_t Vy_recv = *(data_buf + 8) << 8 | *(data_buf + 7);
		int16_t Vz_recv = *(data_buf + 10) << 8 | *(data_buf + 9);
		uint8_t adjust = parm & 0x80;
		DEBUG("motion: 0x%02X, %d, %d, %d\n", parm, Vx_recv, Vy_recv, Vz_recv);

		if (Vx_recv == 0 && Vy_recv == 0 && Vz_recv == 0)
		{
			Motion_Stop(STOP_BRAKE);
		}
		else
		{
			Motion_Ctrl(Vx_recv, Vy_recv, Vz_recv, (adjust==0?0:1));
		}
		break;
	}

	/* Check function byte: PID parameter setting */
	case FUNC_SET_MOTOR_PID:
	{
		uint16_t kp_recv = *(data_buf + 5) << 8 | *(data_buf + 4);
		uint16_t ki_recv = *(data_buf + 7) << 8 | *(data_buf + 6);
		uint16_t kd_recv = *(data_buf + 9) << 8 | *(data_buf + 8);
		uint8_t args = *(data_buf + 10);
		float kp = kp_recv / 1000.0;
		float ki = ki_recv / 1000.0;
		float kd = kd_recv / 1000.0;
		DEBUG("pid:%.2f, %.2f, %.2f, args:0x%02X\n", kp, ki, kd, args);
		PID_Set_Motor_Parm(MAX_MOTOR, kp, ki, kd);
		if (args == SAVE_VERIFY)
		{
			Flash_Set_PID(MAX_MOTOR, kp, ki, kd);
		}
		// uint8_t forever = args & 0x0F;
		// uint8_t motor_x = (args >> 4) & 0x0F;
		// if (motor_x <= MAX_MOTOR)
		// {
		// 	uint8_t id = MAX_MOTOR;
		// 	if(motor_x > 0)
		// 	{
		// 		id = motor_x - 1;
		// 	}
		// 	PID_Set_Motor_Parm(id, kp, ki, kd);
		// 	if (forever == 0x0F)
		// 	{
		// 		Flash_Set_PID(id, kp, ki, kd);
		// 	}
		// }
		break;
	}

	/* Check function byte: yaw PID parameter setting */
	case FUNC_SET_YAW_PID:
	{
		uint16_t kp_recv = *(data_buf + 5) << 8 | *(data_buf + 4);
		uint16_t ki_recv = *(data_buf + 7) << 8 | *(data_buf + 6);
		uint16_t kd_recv = *(data_buf + 9) << 8 | *(data_buf + 8);
		uint8_t forever = *(data_buf + 10);
		float kp = kp_recv / 1000.0;
		float ki = ki_recv / 1000.0;
		float kd = kd_recv / 1000.0;
		DEBUG("YAW PID:%.2f, %.2f, %.2f, args:0x%02X\n", kp, ki, kd, forever);
		PID_Yaw_Set_Parm(kp, ki, kd);
		if (forever == SAVE_VERIFY)
		{
			Flash_Set_Yaw_PID(kp, ki, kd);
		}
		break;
	}

	/* Check function byte: set the car type */
	case FUNC_CAR_TYPE:
	{
		uint8_t carType = *(data_buf + 4);
		uint8_t verify = *(data_buf + 5);
		DEBUG("SET_CAR_TYPE=0x%02X, 0x%02X", carType, verify);
		if (verify == SAVE_VERIFY)
		{
			if (carType < CAR_TYPE_MAX)
			{
				Motion_Set_Car_Type((car_type_t)carType);
				Flash_Set_CarType(carType);
				if (Motion_Get_Car_Type() ==  CAR_SUNRISE)
				{
					PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
					Flash_Set_PID(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
				}
			}
		}
		else
		{
			if (carType < CAR_TYPE_MAX)
			{
				Motion_Set_Car_Type((car_type_t)carType);
				if (Motion_Get_Car_Type() ==  CAR_SUNRISE)
				{
					PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
				}
			}
		}
		break;
	}

	/* Check function byte: serial bus servo control */
	case FUNC_UART_SERVO:
	{
		uint8_t id = *(data_buf + 4);
		uint16_t value = *(data_buf + 6) << 8 | *(data_buf + 5);
		uint16_t time = *(data_buf + 8) << 8 | *(data_buf + 7);
		DEBUG("uartServo:%d, %d, %d\n", id, value, time);
		UartServo_Ctrl(id, value, time);
		break;
	}

	/* Check function byte: set the serial bus servo ID */
	case FUNC_UART_SERVO_ID:
	{
		uint8_t id = *(data_buf + 4);
		DEBUG("setid:%d\n", id);
		UartServo_Set_ID(id);
		break;
	}

	/* Check function byte: enable or disable the serial bus servo torque */
	case FUNC_UART_SERVO_TORQUE:
	{
		uint8_t enable = *(data_buf + 4);
		DEBUG("torque:%d\n", enable);
		UartServo_Set_Torque(enable);
		break;
	}

	/* Check function byte: control the robotic arm */
	case FUNC_ARM_CTRL:
	{
		uint16_t value1 = *(data_buf + 5) << 8 | *(data_buf + 4);
		uint16_t value2 = *(data_buf + 7) << 8 | *(data_buf + 6);
		uint16_t value3 = *(data_buf + 9) << 8 | *(data_buf + 8);
		uint16_t value4 = *(data_buf + 11) << 8 | *(data_buf + 10);
		uint16_t value5 = *(data_buf + 13) << 8 | *(data_buf + 12);
		uint16_t value6 = *(data_buf + 15) << 8 | *(data_buf + 14);
		uint16_t t_time = *(data_buf + 17) << 8 | *(data_buf + 16);
		DEBUG("arm:%d, %d, %d, %d, %d, %d, %d\n", value1, value2, value3, value4, value5, value6, t_time);
		UartServo_Set_Snyc_Buffer(value1, value2, value3, value4, value5, value6);
		UartServo_Sync_Write(t_time);
		break;
	}

	/* Check function byte: robotic arm median offset setting */
	case FUNC_ARM_OFFSET:
	{
		uint16_t s_id = *(data_buf + 4);
		DEBUG("arm offset:%d\n", s_id);
		if (s_id == 0)
		{
			Flash_Reset_ARM_Median_Value();
			Request_Data(FUNC_ARM_OFFSET, s_id);
		}
		else if (s_id <= MAX_SERVO_NUM)
		{
			UartServo_Set_Read_State(ARM_READ_TO_FLASH);
			Request_Data(FUNC_ARM_OFFSET, s_id);
		}
		break;
	}


	/* Check function byte: set the default angle of the Ackermann-type car */
	case FUNC_AKM_DEF_ANGLE:
	{
		uint8_t id = *(data_buf + 4);
		uint8_t angle = *(data_buf + 5);
		uint8_t state = *(data_buf + 6);
		DEBUG("akm def angle:%d, %d, %d\n", id, angle, state);
		if (state == SAVE_VERIFY)
		{
			if (id == 1)
			{
				Ackerman_Set_Default_Angle(angle, 1);
			}
		}
		else
		{
			if (id == 1)
			{
				Ackerman_Set_Default_Angle(angle, 0);
			}
		}
		break;
	}

	/* Check function byte: turn the front-wheel servo of the Ackermann car */
	case FUNC_AKM_STEER_ANGLE:
	{
		uint8_t id = *(data_buf + 4);
		int8_t angle = *(data_buf + 5);
		DEBUG("akm steer angle:%d, %d\n", id, angle);
		if (id == 1)
		{
			Ackerman_Steering(angle);
		}
		else if (id == 0x81)
		{
			Ackerman_Steering_with_car(angle);
		}
		break;
	}


	/* Request data */
	case FUNC_REQUEST_DATA:
	{
		uint8_t request = *(data_buf + 4);
		uint8_t param = *(data_buf + 5);
		DEBUG("request=0x%02X, %d\n", request, param);
		Request_Data(request, param);
		break;
	}

	/* Reset the FLASH data */
	case FUNC_RESET_FLASH:
	{
		uint8_t verify = *(data_buf + 4);
		if (verify == SAVE_VERIFY)
		{
			Flash_Reset_All_Value();
			Flash_Init();
			DEBUG("reset flash=%d\n", verify);
			Bsp_Reset_MCU();
		}
		break;
	}

	default:
		break;
	}
}



/**
 * @Brief: Data reception and storage
 * @Note: 
 * @Parm: The single received data byte
 * @Retval: 
 */
void Upper_Data_Receive(uint8_t Rx_Temp)
{
	switch (RxFlag)
	{
	case 0:
		if (Rx_Temp == PTO_HEAD)
		{
			RxBuffer[0] = PTO_HEAD;
			RxFlag = 1;
		}
		break;

	case 1:
		if (Rx_Temp == PTO_DEVICE_ID)
		{
			RxBuffer[1] = PTO_DEVICE_ID;
			RxFlag = 2;
			RxIndex = 2;
		}
		else
		{
			RxFlag = 0;
			RxBuffer[0] = 0x0;
		}
		break;

	case 2:
		New_CMD_length = Rx_Temp + 2;
		if (New_CMD_length >= PTO_MAX_BUF_LEN)
		{
			RxIndex = 0;
			RxFlag = 0;
			RxBuffer[0] = 0;
			RxBuffer[1] = 0;
			New_CMD_length = 0;
			break;
		}
		RxBuffer[RxIndex] = Rx_Temp;
		RxIndex++;
		RxFlag = 3;
		break;

	case 3:
		RxBuffer[RxIndex] = Rx_Temp;
		RxIndex++;
		if (RxIndex >= New_CMD_length)
		{
			New_CMD_flag = 1;
			RxIndex = 0;
			RxFlag = 0;
		}
		break;

	default:
		break;
	}
}


// Execute a CAN control command
void Upper_CAN_Execute_Command(uint8_t func, uint8_t* parm)
{
	DEBUG("CAN CMD:FUNC:0x%02X, Parm:0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n", 
        func, parm[0], parm[1], parm[2], parm[3], parm[4]);
    switch (func)
    {
	/* Auto-upload low-level data */
	case FUNC_AUTO_REPORT:
	{
		uint8_t state = parm[0];
		uint8_t forever = parm[1];
		DEBUG("report:%d, %d\n", state, forever);
		Set_Auto_Report(state);
		// Check whether to permanently save it to flash
		if (forever == SAVE_VERIFY)
		{
			Flash_Set_Auto_Report(state);
		}
		break;
	}

	/* Check function byte: buzzer control */
	case FUNC_BEEP:
    {
		uint16_t time = parm[1] << 8 | parm[0];
		Beep_On_Time(time);
        break;
    }

	/* Check function byte: control a single servo */
	case FUNC_PWM_SERVO:
	{
		uint8_t servo_id = parm[0];
		uint8_t angle = parm[1];
		DEBUG("pwmServo:%d, %d\n", servo_id, angle);
		PwmServo_Set_Angle(servo_id - 1, angle);
		break;
	}

	/* Check function byte: control all servos */
	case FUNC_PWM_SERVO_ALL:
	{
		uint8_t angle_s1 = parm[0];
		uint8_t angle_s2 = parm[1];
		uint8_t angle_s3 = parm[2];
		uint8_t angle_s4 = parm[3];
		DEBUG("all Servo:%d, %d, %d, %d\n", angle_s1, angle_s2, angle_s3, angle_s4);
		PwmServo_Set_Angle_All(angle_s1, angle_s2, angle_s3, angle_s4);
		break;
	}

	/* Check function byte: RGB light control */
	case FUNC_RGB:
	{
		u8 index = parm[0];
		u8 red = parm[1];
		u8 green = parm[2];
		u8 blue = parm[3];
		RGB_Set_Color(index, red, green, blue);
		DEBUG("RGB:%d, %d, %d, %d\n", index, red, green, blue);
		RGB_Update();
		break;
	}

	/* Check function byte: RGB light effect control */
	case FUNC_RGB_EFFECT:
	{
		u8 effect = parm[0];
		u8 speed = parm[1];
		if (app_rgb_get_effect() != effect || effect == 0)
		{
			RGB_Clear();
			RGB_Update();
		}
		app_rgb_set_effect(effect, speed);
		if (effect == RGB_BREATHING)
		{
			u8 breath_color = parm[2];
			app_rgb_set_breathing_color(breath_color);
		}
		break;
	}

	/* Check function byte: reset the car state, including stopping, lights off, buzzer off */
	case FUNC_RESET_STATE:
	{
		u8 verify = parm[0];
		if (verify == SAVE_VERIFY)
		{
			Motion_Ctrl_State(MOTION_BRAKE, 0, 0);
			app_rgb_set_effect(RGB_NO_EFFECT, 5);
			RGB_Clear();
			RGB_Update();
			Beep_On_Time(0);
		}
		break;
	}

	/* Control the motors, encoder not used */
	case FUNC_MOTOR:
	{
		int16_t speed[4] = {0};
		int8_t motor_1 = parm[0];
		int8_t motor_2 = parm[1];
		int8_t motor_3 = parm[2];
		int8_t motor_4 = parm[3];
		DEBUG("motor=%d, %d, %d, %d", motor_1, motor_2, motor_3, motor_4);
		
		int16_t motor_pulse = MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE;
		if (Motion_Get_Car_Type() == CAR_SUNRISE)
		{
			motor_pulse = MOTOR_MAX_PULSE - MOTOR_SUNRISE_IGNORE_PULSE;
		}
		speed[0] = (int16_t)motor_1 * (motor_pulse / 100.0);
		speed[1] = (int16_t)motor_2 * (motor_pulse / 100.0);
		speed[2] = (int16_t)motor_3 * (motor_pulse / 100.0);
		speed[3] = (int16_t)motor_4 * (motor_pulse / 100.0);
		// PWM control of the car motion
		Motion_Set_Pwm(speed[0], speed[1], speed[2], speed[3]);
		break;
	}

	/* Control the car motion */
	case FUNC_CAR_RUN:
    {
        uint8_t state = parm[0];
        uint16_t speed = parm[2] << 8 | parm[1];
        if (speed > 100) speed = 100;
        Motion_Ctrl_State(state, speed, 0);
        break;
    }

	/* Check function byte: car speed setting */
	case FUNC_MOTION:
	{
		uint8_t state = parm[0];
		int16_t Vx_recv = (int8_t)parm[1];
		int16_t Vy_recv = (int8_t)parm[2];
		int16_t Vz_recv = (int8_t)parm[3];
		uint8_t adjust = state & 0x80;
		DEBUG("motion: 0x%02X, %d, %d, %d\n", parm, Vx_recv, Vy_recv, Vz_recv);

		if (Vx_recv == 0 && Vy_recv == 0 && Vz_recv == 0)
		{
			Motion_Stop(STOP_BRAKE);
		}
		else
		{
			Motion_Ctrl(Vx_recv*10, Vy_recv*10, Vz_recv*10, (adjust==0?0:1));
		}
		break;
	}

	/* Check function byte: set the car type */
	case FUNC_CAR_TYPE:
	{
		uint8_t carType = parm[0];
		uint8_t verify = parm[1];
		DEBUG("SET_CAR_TYPE=0x%02X, 0x%02X", carType, verify);
		if (verify == SAVE_VERIFY)
		{
			if (carType < CAR_TYPE_MAX)
			{
				Motion_Set_Car_Type((car_type_t)carType);
				Flash_Set_CarType(carType);
				if (Motion_Get_Car_Type() ==  CAR_SUNRISE)
				{
					PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
					Flash_Set_PID(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
				}
			}
		}
		else
		{
			if (carType < CAR_TYPE_MAX)
			{
				Motion_Set_Car_Type((car_type_t)carType);
				if (Motion_Get_Car_Type() ==  CAR_SUNRISE)
				{
					PID_Set_Motor_Parm(MAX_MOTOR, PID_SUNRISE_KP, PID_SUNRISE_KI, PID_SUNRISE_KD);
				}
			}
		}
		break;
	}

	/* Check function byte: serial bus servo control */
	case FUNC_UART_SERVO:
	{
		uint8_t id = parm[0];
		uint16_t value = parm[2] << 8 | parm[1];
		uint16_t time = parm[4] << 8 | parm[3];
		DEBUG("uartServo:%d, %d, %d\n", id, value, time);
		UartServo_Ctrl(id, value, time);
		break;
	}

	/* Check function byte: set the serial bus servo ID */
	case FUNC_UART_SERVO_ID:
	{
		uint8_t id = parm[0];
		DEBUG("setid:%d\n", id);
		UartServo_Set_ID(id);
		break;
	}

	/* Check function byte: enable or disable the serial bus servo torque */
	case FUNC_UART_SERVO_TORQUE:
	{
		uint8_t enable = parm[0];
		DEBUG("torque:%d\n", enable);
		UartServo_Set_Torque(enable);
		break;
	}

	/* Check function byte: set the default angle of the Ackermann-type car */
	case FUNC_AKM_DEF_ANGLE:
	{
		uint8_t id = parm[0];
		uint8_t angle = parm[1];
		uint8_t state = parm[2];
		DEBUG("akm def angle:%d, %d, %d\n", id, angle, state);
		if (state == SAVE_VERIFY)
		{
			if (id == 1)
			{
				Ackerman_Set_Default_Angle(angle, 1);
			}
		}
		else
		{
			if (id == 1)
			{
				Ackerman_Set_Default_Angle(angle, 0);
			}
		}
		break;
	}

	/* Check function byte: turn the front-wheel servo of the Ackermann car */
	case FUNC_AKM_STEER_ANGLE:
	{
		uint8_t id = parm[0];
		int8_t angle = parm[1];
		DEBUG("akm steer angle:%d, %d\n", id, angle);
		if (id == 1)
		{
			Ackerman_Steering(angle);
		}
		else if (id == 0x81)
		{
			Ackerman_Steering_with_car(angle);
		}
		break;
	}


	/* Request data */
	case FUNC_REQUEST_DATA:
	{
		uint8_t request = parm[0];
		uint8_t param = parm[1];
		DEBUG("request=0x%02X, %d\n", request, param);
		Request_Data(request, param);
		break;
	}

	/* CAN receive buzzer switch */
	case FUNC_CAN_RX_BEEP:
	{
		uint8_t state = parm[0];
		DEBUG("can rx beep=0x%02X\n", state);
		CAN_RX_Beep(state);
		break;
	}

	/* OLED display flag */
	case FUNC_OLED_FLAG:
	{
		uint8_t state = parm[0];
		DEBUG("oled flag=0x%02X\n", state);
		App_Set_OLED_Flag(state);
		break;
	}

	/* Reset the FLASH data */
	case FUNC_RESET_FLASH:
	{
		uint8_t verify = parm[0];
		if (verify == SAVE_VERIFY)
		{
			Flash_Reset_All_Value();
			Flash_Init();
			DEBUG("reset flash=%d\n", verify);
			Bsp_Reset_MCU();
		}
		break;
	}

	/* Test mode */
	case FUNC_TESE_MODE:
	{
		uint8_t enable = parm[0];
		uint8_t verify = parm[1];
		DEBUG("test mode=%d, %d\n", enable, verify);
		if (verify == SAVE_VERIFY)
		{
			if (enable == MODE_TEST)
			{
				DEBUG("Start Test Mode\r\n");
				Bsp_Set_TestMode(MODE_TEST);
				App_Test_Mode_Init();
			}
			else
			{
				Bsp_Set_TestMode(MODE_STANDARD);
			}
		}
		break;
	}

	/* Clear the yaw display */
	case FUNC_TEST_CLEAR_YAW:
	{
		uint8_t verify = parm[0];
		if (verify == 1)
		{
			App_Clear_Yaw();
			DEBUG("test clear yaw=%d\n", verify);
		}
		break;
	}
    
    default:
        break;
    }
}

