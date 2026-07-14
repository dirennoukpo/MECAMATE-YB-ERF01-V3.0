#ifndef __APP_BAT_H__
#define __APP_BAT_H__

#include "stm32f10x.h"

typedef enum Battery_State
{
    BATTERY_LOW,          // Battery voltage too low
    BATTERY_NORMAL,       // Battery voltage normal
    BATTERY_OVER_VOLTAGE  // Battery voltage too high
} Battery_State_t;


uint8_t System_Enable(void);
uint8_t Bat_State(void);
int Bat_Voltage_Z10(void);
uint8_t Bat_Get_Low_Voltage(void);
uint8_t Bat_Get_Over_Voltage(void);
uint8_t Bat_Show_LED_Handle(uint8_t enable_beep);

#endif /* __APP_BAT_H__ */
