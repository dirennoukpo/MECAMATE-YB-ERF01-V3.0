#include "app_bat.h"
#include "app.h"
#include "app_oled.h"
#include "app_motion.h"
#include "app_rgb.h"

#include "bsp.h"

// Battery voltage abnormality count threshold. Bat_State() runs every 100 ms (vTask_App
// ticks at 10 ms and Bat_Show_LED_Handle acts one call in ten), and the test below is a
// strict '>', so the alarm needs BAT_CHECK_COUNT + 1 = 21 CONSECUTIVE abnormal samples:
// 2.1 s, not the 1 s the stock comment claimed.
#define BAT_CHECK_COUNT        20

uint8_t g_system_enable = 1;      // System function state. Becomes 0 after an undervoltage or overvoltage is detected. Can only be restored to 1 by a reset
uint8_t g_bat_state = 1;          // Battery low-voltage state. Default is 1, becomes 0 when undervoltage is detected and 2 when overvoltage is detected. Can only be restored to 1 by a reset.
int Voltage_Z10 = 0;              // Battery voltage value
int Voltage_Unusual_Count = 0;    // Abnormal voltage count


// Check the battery voltage state. Return value: 1=voltage normal, 0=voltage too low, 2=voltage too high.
//
// PORT NOTE -- this had two bodies, picked at run time by Motion_Get_Car_Type(), because the
// CHASSIS type doubled as a declaration of the PACK voltage. The pack is now declared for what
// it is (BATTERY_CELLS, config.h), so one body is enough. That removes two teeth:
//
//   (1) THE RATCHET. The non-SUNRISE body carried a clause that skipped the alarm between
//       6.5 V and 8.5 V -- but that clause neither incremented Voltage_Unusual_Count NOR
//       reset it. The only reset sat in the final else, reachable only inside 9.6-13.0 V,
//       where a 2S pack never goes. So on a 2S the counter could only ever climb: every
//       isolated sag below 6.5 V (four mecanum motors stalling) added one, permanently.
//       Twenty such sags, hours apart, and the twenty-first latched BATTERY_LOW on a
//       perfectly healthy pack -- brake on, robot dead until a power cycle. The counter is
//       now cleared by any in-band sample, so BAT_CHECK_COUNT means what its name says:
//       21 CONSECUTIVE bad samples.
//
//   (2) THE BLIND WINDOW ITSELF. Harmless on a 2S (which lives there), lethal on a 3S:
//       6.5-8.5 V is 2.2-2.8 V/cell, and the firmware answered BATTERY_NORMAL.
//
// Side effect, and a welcome one: a forged CAN/UART FUNC_CAR_TYPE frame can no longer move
// the battery thresholds at run time.
static uint8_t Bat_Check_Voltage(int Voltage)
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
	return BATTERY_NORMAL;
}

// Low-voltage alarm threshold, in tenths of a volt. Follows the declared PACK
// (BATTERY_CELLS in config.h), no longer the chassis type.
uint8_t Bat_Get_Low_Voltage(void)
{
	return (uint8_t)BAT_LOW_VOLTAGE_Z10;
}

// Over-voltage alarm threshold, in tenths of a volt. Doubles as the "wrong pack plugged in"
// detector: on a 2S build, any healthy 3S reads far above it.
uint8_t Bat_Get_Over_Voltage(void)
{
	return (uint8_t)BAT_OVER_VOLTAGE_Z10;
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

