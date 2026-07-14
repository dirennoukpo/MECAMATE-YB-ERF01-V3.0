#include "bsp_usart.h"
#include "bsp.h"
#include "app.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "app_oled.h"
#include "app_motion.h"
#include "app_uart_servo.h"
#include "app_pid.h"
#include "app_bat.h"
#include "app_sbus.h"
#include "app_mecanum.h"
#include "app_rgb.h"
#include "app_flash.h"

#include "icm20948.h"
#include "app_math.h"


// One count represents ten milliseconds
uint16_t g_update = 0;
uint16_t g_oled_count = 0;
uint8_t g_oled_flag = OLED_MAX_FLAG;

/* Auto report flag */
uint16_t g_Auto_Report = ENABLE_AUTO_REPORT;
uint8_t g_system_init = 0;
uint8_t g_enable_beep = 1;
float last_yaw = 0;

uint8_t App_FreeRTOS_Enable(void)
{
	return g_system_init;
}

// Delay XX milliseconds; FreeRTOS must be started before calling this function
void App_Delay_ms(uint16_t ms)
{
	vTaskDelay(ms / portTICK_RATE_MS);
}


// Set automatic data reporting
void Set_Auto_Report(uint16_t enable)
{
	if (enable == 0)
	{
		g_Auto_Report = 0;
		return;
	}
	g_Auto_Report = 1;
}

// Set the OLED display flag, flag=1 shows gyroscope data, flag=2 shows voltage data.
void App_Set_OLED_Flag(uint8_t flag)
{
	g_oled_flag = flag;
}

void App_Clear_Yaw(void)
{
	if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
	{
		last_yaw = ICM20948_Get_Yaw_Now()*RtA;
	}
	else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
	{
		last_yaw = MPU_Get_Yaw_Now()*RtA;
	}
}

// Test mode initialization
void App_Test_Mode_Init(void)
{
	Beep_On_Time(100);
	CAN_RX_Beep(0);
	Motion_Stop(1);
	UartServo_Ctrl(0xFE, 2000, 100);
	OLED_Show_Test_Mode();
}


// Initialize the APP content
void App_Init(void)
{
	PID_Param_Init();
	app_rgb_init();
	Flash_Init();

	#if ENABLE_CAR_SUNRISE_ONLY
	Motion_Set_Car_Type(CAR_SUNRISE);
	#endif

	#if PID_ASSISTANT_EN
	protocol_init();
	int32_t target_location = 500;
	int32_t cycle = 10;
	set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);    // Synchronize the start button state of the host
	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_location, 1);     // Send the target value to channel 1
	set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &cycle, 1);
	USART1_Send_ArrayU8("\n", 1);
	#endif
	
	#if ENABLE_ICM20948
	if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
	{
		DEBUG("Start ICM20948 Init\r\n");
		int result = ICM_20948_Init();
		if (result)
		{
			printf("ICM_INIT ERROR:%d\n", result);
			OLED_Show_Error();
			#if ENABLE_IMU_ERROR_PASS
			Bsp_Imu_Type_None();
			#else
			Bsp_Long_Beep_Alarm();
			while(1);
			#endif
		}
		DEBUG("ICM_INIT OK\n");
	}
	#endif

	CAN_RX_Beep(1);
	g_system_init = 1;
}

// Logic handling, called once every 10MS
void App_Loop(void)
{
	Beep_Timeout_Close_Handle();
	app_rgb_effects_handle();
}

// Send the current firmware version number
void App_Send_Version(void)
{
	Bsp_Send_Version();
}

/* 10ms speed measurement thread */
void vTask_Speed(void *pvParameters)
{
	#if ENABLE_USART2 & !ENABLE_SBUS
	App_Delay_ms(2000);
	USART2_Connect_BT();
	#endif
	uint32_t lastWakeTime = xTaskGetTickCount();
	while (System_Enable())
	{
		// App_Delay_ms(10);
		vTaskDelayUntil(&lastWakeTime, 10);
		Encoder_Update_Count();
		Motion_Handle();
	}
	Motion_Stop(STOP_FREE);
	App_Delay_ms(40);
	DEBUG("Finish vTask_Speed\n");
	while (1)
	{
		App_Delay_ms(1000);
	}
	
}

/* APP event handling */
void vTask_App_Handle(void *pvParameters)
{
	// BEEP_ON();
	// App_Delay_ms(100);
	// BEEP_OFF();
	Beep_On_Time(100);
	while (1)
	{
		App_Delay_ms(10);
		#if ENABLE_IWDG
		IWDG_Feed();
		#endif
		// If the battery is low or the voltage is too high, skip the APP_Loop below
		if (Bat_Show_LED_Handle(g_enable_beep))
		{
			App_Loop();
			LED_SW_ON();
		}
	}
}

/* Parse the commands sent by the host, and process the control content */
void vTask_Control(void *pvParameters)
{
	while (System_Enable())
	{
		if (Get_CMD_Flag())
		{
			Upper_Data_Parse(Get_RxBuffer(), Get_CMD_Length());
			Clear_CMD_Flag();
		}
		if (Get_Request_Flag() > 0)
		{
			Send_Request_Data();
		}
		SBUS_Handle();
		App_Delay_ms(1);
	}
	g_system_init = 0;
	while (1)
	{
		if (Get_CMD_Flag())
		{
			Upper_Data_Parse_Low_Battery(Get_RxBuffer(), Get_CMD_Length());
			Clear_CMD_Flag();
		}
		if (Get_Request_Flag() > 0)
		{
			Send_Request_Data();
		}
		App_Delay_ms(1);
	}
}

// Automatic data reporting thread
void vTask_Auto_Report(void *pvParameters)
{
	App_Delay_ms(AUTO_SEND_TIMEOUT);
	uint16_t report_count = 0;
	while (System_Enable())
	{
		if (g_Auto_Report)
		{
			if (report_count == 1)
			{
				Motion_Send_Data();
			}
			else if (report_count == 11)
			{
				if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
				{
					ICM20948_Send_Raw_Data();
				}
				else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
				{
					MPU9250_Send_Raw_Data();
				}
			}
			else if (report_count == 21)
			{
				if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
				{
					ICM20948_Send_Attitude_Data();
				}
				else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
				{
					MPU9250_Send_Attitude_Data();
				}
			}
			else if (report_count == 31)
			{
				Encoder_Send_Count_Now();
			}
		}
		report_count++;
		if (report_count > AUTO_SEND_TIMEOUT) report_count = 0;
		App_Delay_ms(1);
		
		#if PID_ASSISTANT_EN
		receiving_process();
		#endif
	}
	App_Delay_ms(10);
	DEBUG("Finish vTask_Auto_Report\n");
	while (1)
	{
		App_Delay_ms(1000);
	}
}

/* OLED display task */
void vTask_OLED(void *pvParameters)
{
	OLED_Show_CarType(Motion_Get_Car_Type(), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
	while (System_Enable())
	{
		g_oled_count++;
		switch (g_oled_flag)
		{
		case OLED_FLAG_NO_DISPLAY:
		{
			OLED_Clear();
			OLED_Refresh();
			g_oled_flag = OLED_MAX_FLAG;
			break;
		}
		case OLED_FLAG_IMU:
		{
			if (g_oled_count >= 5)
			{
				g_oled_count = 0;
				#if ENABLE_DEBUG_ICM_ATT
				float yaw = 0, roll = 0, pitch = 0;
				yaw = ICM20948_Get_Yaw_Now()*RtA;
				roll = ICM20948_Get_Roll_Now()*RtA;
				pitch = ICM20948_Get_Pitch_Now()*RtA;
				OLED_Show_IMU_Attitude(yaw, roll, pitch);
				printf("YAW:%.2f, ROLL:%.2f, Pitch:%.2f\n", yaw, roll, pitch);
				#else
				if (Bsp_Get_TestMode() == MODE_TEST)
				{
					if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
					{
						OLED_Show_YAW(ICM20948_Get_Yaw_Now()*RtA-last_yaw);
					}
					else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
					{
						OLED_Show_YAW(MPU_Get_Yaw_Now()*RtA-last_yaw);
					}
				}
				else
				{
					OLED_Show_Test_Mode_Error();
				}
				#endif
			}
			break;
		}
		case OLED_FLAG_VOLTAGE:
		{
			if (Bsp_Get_TestMode() == MODE_TEST)
			{
				if (g_oled_count >= 100)
				{
					g_oled_count = 0;
					OLED_Show_Voltage(Bat_Voltage_Z10());
				}
			}
			else
			{
				OLED_Show_Test_Mode_Error();
			}
			break;
		}
		case OLED_FLAG_MOTOR_SPEED:
		{
			if (Bsp_Get_TestMode() == MODE_TEST)
			{
				float speed_f[4] = {0};
				Motion_Get_Motor_Speed(speed_f);
				OLED_Show_Motor_Speed(-speed_f[0], -speed_f[1], speed_f[2], speed_f[3]);
			}
			else
			{
				OLED_Show_Test_Mode_Error();
			}
			break;
		}
		case OLED_FLAG_UART_SERVO:
		{
			if (Bsp_Get_TestMode() == MODE_TEST)
			{
				OLED_Show_UART_Servo_Read(UartServo_Test_Read_State());
			}
			else
			{
				OLED_Show_Test_Mode_Error();
			}
			break;
		}
		case OLED_MAX_FLAG:
		{
			g_oled_count = 100;
			App_Clear_Yaw();
			break;
		}
		default:
		{
			g_oled_flag = OLED_FLAG_NO_DISPLAY;
			break;
		}
		}
		App_Delay_ms(10);
	}
	App_Delay_ms(20);
	DEBUG("Finish vTask_OLED\n");
	while (1)
	{
		App_Delay_ms(1000);
	}
}

// Key event handling
void vTask_Key(void *pvParameters)
{
	#if ENABLE_KEY_RELEASE
	while (1)
	{
		if (Key1_State(KEY_MODE_ONE_TIME))
		{
			Beep_On_Time(50);
			DEBUG("KEY1 PRESS\n");
			if (System_Enable())
			{
				static uint8_t effect1 = 5;
				RGB_Clear();
				RGB_Update();
				effect1 = (effect1 + 1) % RGB_MAX_EFFECT;
				app_rgb_set_effect(effect1, 5);
			}
			else
			{
				g_enable_beep = 0;
				BEEP_OFF();
			}
		}
		
		if (Key1_Long_Press(10)) // Long press 10 seconds to clear the Flash
		{
			if (System_Enable())
			{
				Beep_On_Time(1000);
				Motion_Stop(STOP_BRAKE);
				app_rgb_set_effect(0, 5);
				RGB_Clear();
				RGB_Update();

				Flash_Reset_All_Value();
				App_Delay_ms(500);
				Bsp_Reset_MCU();
			}
		}
		App_Delay_ms(10);
	}
	#else
	static int key_state = 1;
	while (1)
	{
		if (Key1_State(KEY_MODE_ONE_TIME))
		{
			static u8 effect1 = 0;
			Beep_On_Time(50);
			g_oled_count = 100;
			DEBUG("KEY1 PRESS\n");
			if (key_state)
			{
				key_state = 0;
				// g_oled_flag = OLED_FLAG_IMU;
				// Motion_Ctrl_State(MOTION_RUN, 500, 1);
				effect1 = 6;
			}
			else
			{
				key_state = 1;
				// g_oled_flag = OLED_FLAG_VOLTAGE;
				// Motion_Stop(1);
				effect1 = 0;
			}
			if(!System_Enable()) g_enable_beep = 0;

			CAN_Test_Send();
			App_Clear_Yaw();

			RGB_Clear();
			RGB_Update();
			// effect1 = (effect1 + 1) % 2;
			app_rgb_set_effect(effect1, 5);
		}
		
		if (Key1_Long_Press(10)) // Long press 10 seconds to clear the Flash
		{
			Beep_On_Time(1000);
			Motion_Stop(STOP_BRAKE);
			app_rgb_set_effect(0, 5);
			RGB_Clear();
			RGB_Update();

			Flash_Reset_All_Value();
			Flash_Init();
		}
		App_Delay_ms(10);
	}
	#endif
}

// Handle the attitude sensor data
void vTask_IMU(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while (System_Enable())
	{
		if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
		{
			ICM20948_Read_Data_Handle();
		}
		else if (Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
		{
			MPU9250_Read_Data_Handle();
		}
		// App_Delay_ms(10);
		vTaskDelayUntil(&lastWakeTime, 10);
	}
	App_Delay_ms(30);
	DEBUG("Finish vTask_IMU\n");
	while (1)
	{
		App_Delay_ms(1000);
	}
	
}


/* Start the FreeRTOS task list, the content after this function is no longer executed */
void App_Start_FreeRTOS(void)
{
	xTaskCreate(vTask_Speed, "Task Speed", 512, NULL, 10, NULL);
	printf("start vTask_Speed\n");
	// Stack raised from 128 to 512 words for the GCC/newlib port. This is NOT a
	// comfort tweak: on the FUNC_RESET_FLASH command, this task descends down to
	// Upper_Data_Parse -> Flash_Init -> printf("FRead PID:%.3f...") (app_flash.c:155).
	// newlib's floating-point formatting (_vfiprintf_r 152 B + _printf_float 104 B +
	// _dtoa_r 120 B + _Balloc/_malloc_r) costs ~808 B measured at disassembly, whereas
	// MicroLIB fitted in ~230 B with no malloc. The original 128 words (512 B), which
	// were enough under Keil, therefore overflow silently (configCHECK_FOR_STACK_OVERFLOW
	// is not defined) and corrupt the neighbouring heap_2 block.
	xTaskCreate(vTask_Control, "Task Control", 512, NULL, 9, NULL);
	printf("start vTask_Control\n");
	xTaskCreate(vTask_Key, "Task KEY", 128, NULL, 8, NULL);
	printf("start vTask_Key\n");
	xTaskCreate(vTask_Auto_Report, "Task Report", 512, NULL, 7, NULL);
	printf("start vTask_Auto_Report\n");
	xTaskCreate(vTask_App_Handle, "Task App_Handle", 128, NULL, 4, NULL);
	printf("start vTask_App_Handle\n");
	xTaskCreate(vTask_IMU, "Task MPU", 1024, NULL, 3, NULL);
	printf("start vTask_IMU\n");
	#if ENABLE_OLED
	xTaskCreate(vTask_OLED, "Task OLED", 512, NULL, 1, NULL);
	printf("start vTask_OLED\n");
	#endif
	
	printf("start vTaskStartScheduler\n");
	vTaskStartScheduler();
}
