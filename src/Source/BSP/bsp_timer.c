#include "bsp_timer.h"
#include "bsp_pwmServo.h"


/**************************************************************************
Function: TIM6 initialization, 10 ms timer
Entry parameters: none
Return  value: none
**************************************************************************/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //Enable the timer clock
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;			 // Prescaler
	TIM_TimeBaseStructure.TIM_Period = 99;				 //Set the counter auto-reload value
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);                //Clear the TIM update flag
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	//Interrupt priority NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			  //TIM1 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //Preemption priority level 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //Sub-priority level 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ channel enabled
	NVIC_Init(&NVIC_InitStructure);							  //Initialize the NVIC registers

	TIM_Cmd(TIM6, ENABLE);
}


/**************************************************************************
Function: TIM7 initialization, 10 us timer, software PWM, controls servos and headlights
Entry parameters: none
Return  value: none
**************************************************************************/
void TIM7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //Enable the timer clock
	TIM_TimeBaseStructure.TIM_Prescaler = 71;			 // Prescaler
	TIM_TimeBaseStructure.TIM_Period = 9;				 //Set the counter auto-reload value
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);               //Clear the TIM update flag
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	//Interrupt priority NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;			  //TIM interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //Preemption priority level 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  //Sub-priority level 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ channel enabled
	NVIC_Init(&NVIC_InitStructure);							  //Initialize the NVIC registers

	TIM_Cmd(TIM7, ENABLE);
}


// TIM7 interrupt
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) //Check whether the TIM update interrupt occurred
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);    //Clear the TIMx update interrupt flag
		PwmServo_Handle();
	}
}
