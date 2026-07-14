#include "bsp_beep.h"

uint16_t beep_on_time = 0;
uint8_t beep_state = 0;

void Beep_GPIO_Init(void)
{
	/*Define a structure of type GPIO_InitTypeDef*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*Enable the port clock of the GPIO controlling the buzzer*/
	RCC_APB2PeriphClockCmd(BEEP_GPIO_CLK, ENABLE);
	/*Select the GPIO used to control the buzzer*/
	GPIO_InitStructure.GPIO_Pin = BEEP_GPIO_PIN;
	/*Set the GPIO mode to general-purpose push-pull output*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*Set the GPIO speed to 50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*Call the library function to initialize the GPIO controlling the buzzer*/
	GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStructure);
	/* Turn off the buzzer*/
	// GPIO_ResetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);
	/* Turn on the buzzer*/
	GPIO_SetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);
}

// Refresh the buzzer on-time
static void Beep_Set_Time(uint16_t time)
{
	beep_on_time = time;
}

// Get the remaining on-time of the buzzer
static uint16_t Beep_Get_Time(void)
{
	return beep_on_time;
}

// Refresh the buzzer state
static void Beep_Set_State(uint8_t state)
{
	beep_state = state;
}

// Get the buzzer state
static uint8_t Beep_Get_State(void)
{
	return beep_state;
}

// Set the buzzer on-time: time=0 turns it off, time=1 keeps it on continuously, time>=10 turns it off automatically after xx milliseconds
void Beep_On_Time(uint16_t time)
{
	if (time == BEEP_STATE_ON_ALWAYS)
	{
		Beep_Set_State(BEEP_STATE_ON_ALWAYS);
		Beep_Set_Time(0);
		BEEP_ON();
	}
	else if (time == BEEP_STATE_OFF)
	{
		Beep_Set_State(BEEP_STATE_OFF);
		Beep_Set_Time(0);
		BEEP_OFF();
	}
	else
	{
		if (time >= 10)
		{
			Beep_Set_State(BEEP_STATE_ON_DELAY);
			Beep_Set_Time(time / 10);
			BEEP_ON();
		}
	}
}

/* Buzzer timeout auto-off routine, called once every 10 milliseconds */
void Beep_Timeout_Close_Handle(void)
{
	if (Beep_Get_State() == BEEP_STATE_ON_DELAY)
	{
		if (Beep_Get_Time())
		{
			beep_on_time--;
		}
		else
		{
			BEEP_OFF();
			Beep_Set_State(BEEP_STATE_OFF);
		}
	}
}
