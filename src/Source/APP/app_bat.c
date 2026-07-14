#include "app_bat.h"
#include "app.h"
#include "app_oled.h"
#include "app_motion.h"
#include "app_rgb.h"

#include "bsp.h"

// Battery voltage abnormality count threshold; multiplied by 100 milliseconds gives the delay time, in milliseconds.
// For example: 100*10=1000, i.e. 1 second.
#define BAT_CHECK_COUNT        20

uint8_t g_system_enable = 1;      // System function state. Becomes 0 after an undervoltage or overvoltage is detected. Can only be restored to 1 by a reset
uint8_t g_bat_state = 1;          // Battery low-voltage state. Default is 1, becomes 0 when undervoltage is detected and 2 when overvoltage is detected. Can only be restored to 1 by a reset.
int Voltage_Z10 = 0;              // Battery voltage value
int Voltage_Unusual_Count = 0;    // Abnormal voltage count


// Check the battery voltage state. Return value: 1=voltage normal, 0=voltage too low, 2=voltage too high.
static uint8_t Bat_Check_Voltage(int Voltage)
{
	if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		if (Voltage > Bat_Get_Over_Voltage())
		{
			Voltage_Unusual_Count++;
			if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
			{
				return BATTERY_OVER_VOLTAGE;
			}
		}
		else if (Voltage < Bat_Get_Low_Voltage())
		{
			Voltage_Unusual_Count++;
			if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
			{
				return BATTERY_LOW;
			}
		}
		else
		{
			Voltage_Unusual_Count = 0;
		}
	}
	else
	{
		if (Voltage > Bat_Get_Over_Voltage())
		{
			Voltage_Unusual_Count++;
			if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
			{
				return BATTERY_OVER_VOLTAGE;
			}
		}
		else if (Voltage < Bat_Get_Low_Voltage())
		{
			// Filter out the low-voltage alarm function between 6.5 and 8.5.
			if (Voltage > 85 || Voltage < 65)
			{
				Voltage_Unusual_Count++;
				if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
				{
					return BATTERY_LOW;
				}
			}
		}
		else
		{
			Voltage_Unusual_Count = 0;
		}
	}
	return BATTERY_NORMAL;
}

// The Rosmaster series cars use a 12.6V battery pack, low-voltage alarm threshold is 9.6V.
// The sunrise series cars use an 8.4V battery pack, low-voltage alarm threshold is 6.5V.
uint8_t Bat_Get_Low_Voltage(void)
{
	if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		return 65;
	}
	return 96;
}

// The Rosmaster series cars use a 12.6V battery pack, overvoltage alarm threshold is 13.0V.
// The sunrise series cars use an 8.4V battery pack, overvoltage alarm threshold is 8.5V.
uint8_t Bat_Get_Over_Voltage(void)
{
	if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		return 85;
	}
	return 130;
}


// Query the battery voltage state: returns 0 if a value below 9.6V is read for several consecutive seconds, returns 1 if above 9.6V
uint8_t Bat_State(void)
{
	if (g_bat_state == BATTERY_NORMAL)
	{
		Voltage_Z10 = (int) (Adc_Get_Battery_Volotage() * 10);
		#if ENABLE_LOW_BATTERY_ALARM
		g_bat_state = Bat_Check_Voltage(Voltage_Z10);
		if(g_bat_state != BATTERY_NORMAL)
		{
			g_system_enable = 0;
		}
		#endif
	}
    // DEBUG("BAT:%d, %d", g_bat_state, Voltage_Z10);
	return g_bat_state;
}

int Bat_Voltage_Z10(void)
{
	return Voltage_Z10;
}


// Return whether the system power supply is normal: returns 1 if normal, 0 if abnormal
uint8_t System_Enable(void)
{
	return g_system_enable;
}

// Called every 10 milliseconds. Shows the LED state according to the battery and returns the system state.
uint8_t Bat_Show_LED_Handle(uint8_t enable_beep)
{
	static uint16_t bat_led_state = 0;
	bat_led_state++;
	if (bat_led_state >= 10)
	{
		static uint8_t alarm = 1;
		uint8_t battery_state = Bat_State();
		bat_led_state = 0;
		// NOTE: the OLED alarm texts below are commented out on request -- the screen is
		// dedicated to the battery voltage and must show nothing else. The ALARMS
		// THEMSELVES ARE UNTOUCHED: the buzzer still sounds (Bsp_Led_Show_*), the RGB
		// strip still turns red on over-voltage, and the car still brakes.
		// Since a low battery drives g_system_enable to 0, vTask_OLED leaves its loop and
		// the screen freezes on the last voltage displayed -- which is arguably more
		// useful than the word "Battery Low": you get to read how low it actually got.
		if (battery_state == BATTERY_LOW)
		{
			Bsp_Led_Show_Low_Battery(enable_beep);
			if (alarm)
			{
				alarm = 0;
				// OLED_Draw_Line("Battery Low", 2, true, true);
			}
			Motion_Stop(STOP_BRAKE);
		}
		else if (battery_state == BATTERY_OVER_VOLTAGE)
		{
			Bsp_Led_Show_Overvoltage_Battery(enable_beep);
			if (alarm)
			{
				alarm = 0;
				app_rgb_set_effect(0, 0);
				RGB_Set_Color(RGB_CTRL_ALL, 255, 0, 0);
				RGB_Update();
				// OLED_Draw_Line("Battery", 2, true, false);
				// OLED_Draw_Line("Over Voltage", 3, false, true);
			}
			Motion_Stop(STOP_BRAKE);
		}
		else
		{
			Bsp_Led_Show_State();
		}
	}
	return g_system_enable;
}

