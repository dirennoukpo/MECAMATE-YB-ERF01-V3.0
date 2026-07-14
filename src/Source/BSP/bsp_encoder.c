#include "bsp_encoder.h"
#include "bsp.h"

#include "protocol.h"
#include "app_motion.h"


int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;
int g_Encoder_M3_Now = 0;
int g_Encoder_M4_Now = 0;


// TIM2 initialized in encoder interface mode, 3A 3B
static void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	Bsp_JTAG_Set(SWD_ENABLE);    //=====Enable the SWD interface, disable the JTAG interface. The board's SWD interface can be used for debugging
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // Enable AFIO clock
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);    // remap TIM2 pins CH1 CH2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Enable timer 2 clock


	RCC_APB2PeriphClockCmd(HAL_3A_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_3A_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_3A_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	RCC_APB2PeriphClockCmd(HAL_3B_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_3B_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_3B_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					//Prescaler
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//Set the counter auto-reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//Select the clock division: no division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM counts up
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //Use encoder mode 3

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
    
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);                  //Clear the TIM update flag
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM2, 0);
	//===============================================
	TIM2->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM2, ENABLE);
}

// Timer 3 channel 1 and channel 2 connected to encoder 2A 2B
static void Encoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Enable timer 4 clock

	RCC_APB2PeriphClockCmd(HAL_2A_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_2A_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_2A_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	RCC_APB2PeriphClockCmd(HAL_2B_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_2B_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_2B_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					// Prescaler
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//Set the counter auto-reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//Select the clock division: no division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM counts up
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //Use encoder mode 3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);                   //Clear the TIM update flag
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM3, 0);
	//===============================================
	TIM3->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM3, ENABLE);
}


// TIM4 initialized in encoder interface mode, 4A 4B
static void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //Enable timer 4 clock

	RCC_APB2PeriphClockCmd(HAL_4A_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_4A_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_4A_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	RCC_APB2PeriphClockCmd(HAL_4B_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_4B_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_4B_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					// Prescaler
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//Set the counter auto-reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//Select the clock division: no division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM counts up
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //Use encoder mode 3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);                   //Clear the TIM update flag
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM4, 0);
	//===============================================
	TIM4->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM4, ENABLE);
}

// Timer 5 channel 1 and channel 2 connected to encoder 1A 1B
static void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //Enable timer 5 clock

	RCC_APB2PeriphClockCmd(HAL_1A_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_1A_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_1A_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	RCC_APB2PeriphClockCmd(HAL_1B_CLK, ENABLE);			  //Enable port clock
	GPIO_InitStructure.GPIO_Pin = HAL_1B_PIN;			  //Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Floating input
	GPIO_Init(HAL_1A_PORT, &GPIO_InitStructure);		  //Initialize GPIO with the configured parameters

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					//Prescaler
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//Set the counter auto-reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//Select the clock division: no division
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM counts up
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //Use encoder mode 3

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);                  //Clear the TIM update flag
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM5, 0);
	//===============================================
	TIM5->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM5, ENABLE);
}

/**
 * @Brief: Updated every 10 milliseconds, reads the encoder count
 * @Note: 
 * @Parm: Motor ID number: MOTOR_ID_M1, MOTOR_ID_M2, MOTOR_ID_M3, MOTOR_ID_M4
 * @Retval: Returns the encoder count data
 */
static int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
	int16_t Encoder_TIM = 0;
	switch(Motor_id)
	{
	case MOTOR_ID_M1:  Encoder_TIM = 0x7fff - (short)TIM2 -> CNT; TIM2 -> CNT = 0x7fff; break;
	case MOTOR_ID_M2:  Encoder_TIM = 0x7fff - (short)TIM4 -> CNT; TIM4 -> CNT = 0x7fff; break;
	case MOTOR_ID_M3:  Encoder_TIM = 0x7fff - (short)TIM5 -> CNT; TIM5 -> CNT = 0x7fff; break;
	case MOTOR_ID_M4:  Encoder_TIM = 0x7fff - (short)TIM3 -> CNT; TIM3 -> CNT = 0x7fff; break;
	default:  break;
	}
	return Encoder_TIM;
}


// Return the total encoder count accumulated since power-on (single channel).
int Encoder_Get_Count_Now(uint8_t Motor_id)
{
	if (Motor_id == MOTOR_ID_M1) return g_Encoder_M1_Now;
	if (Motor_id == MOTOR_ID_M2) return g_Encoder_M2_Now;
	if (Motor_id == MOTOR_ID_M3) return g_Encoder_M3_Now;
	if (Motor_id == MOTOR_ID_M4) return g_Encoder_M4_Now;
	return 0;
}

// Get the total counts of the four encoders since power-on.
void Encoder_Get_ALL(int* Encoder_all)
{
	Encoder_all[0] = g_Encoder_M1_Now;
	Encoder_all[1] = g_Encoder_M2_Now;
	Encoder_all[2] = g_Encoder_M3_Now;
	Encoder_all[3] = g_Encoder_M4_Now;
}

// Update the total encoder count values.
void Encoder_Update_Count(void)
{
	if (Motion_Get_Car_Type() == CAR_MECANUM_MAX)
	{
		g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
		// g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1);

		// g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2);
		g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		// g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);
		g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);

		// g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);
		g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);
	}
    else if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
		// g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1);

		// g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2);
		g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		// g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);
		g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);

		// g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);
		g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);
	}
	// Other car types
	else
	{
		// g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
		g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1);

		g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2);
		// g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);
		// g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);

		g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);
		// g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);
	}
}


// Initialize the encoder GPIO and the timer pulse capture
void Encoder_Init(void)
{
	Encoder_Init_TIM2();       // M1 TIM2 PWMC HAL3
	Encoder_Init_TIM4();       // M2 TIM4 PWMD HAL4
	Encoder_Init_TIM5();       // M3 TIM5 PWMA HAL1
	Encoder_Init_TIM3();       // M4 TIM3 PWMB HAL2
}


// Send the current encoder data to the host controller
void Encoder_Send_Count_Now(void)
{
    #define LEN        21
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN-2; // Length
	data_buffer[3] = FUNC_REPORT_ENCODER; // Function byte
	data_buffer[4] = g_Encoder_M1_Now & 0xff;
	data_buffer[5] = (g_Encoder_M1_Now >> 8) & 0xff;
	data_buffer[6] = (g_Encoder_M1_Now >> 16) & 0xff;
	data_buffer[7] = (g_Encoder_M1_Now >> 24) & 0xff;
	data_buffer[8] = g_Encoder_M2_Now & 0xff;
	data_buffer[9] = (g_Encoder_M2_Now >> 8) & 0xff;
	data_buffer[10] = (g_Encoder_M2_Now >> 16) & 0xff;
	data_buffer[11] = (g_Encoder_M2_Now >> 24) & 0xff;
	data_buffer[12] = g_Encoder_M3_Now & 0xff;
	data_buffer[13] = (g_Encoder_M3_Now >> 8) & 0xff;
	data_buffer[14] = (g_Encoder_M3_Now >> 16) & 0xff;
	data_buffer[15] = (g_Encoder_M3_Now >> 24) & 0xff;
	data_buffer[16] = g_Encoder_M4_Now & 0xff;
	data_buffer[17] = (g_Encoder_M4_Now >> 8) & 0xff;
	data_buffer[18] = (g_Encoder_M4_Now >> 16) & 0xff;
	data_buffer[19] = (g_Encoder_M4_Now >> 24) & 0xff;

	for (i = 2; i < LEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}
