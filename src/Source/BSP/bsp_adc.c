#include "bsp_adc.h"
#include "bsp.h"

// ADC initialization interface
void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(BAT_GPIO_CLK | BAT_ADC_CLK, ENABLE); //Enable BAT_ADC channel clock
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);							//Set ADC prescaler factor 6

	//72M/6=12, the ADC max input clock must not exceed 14M
	//PC4 used as analog channel input pin
	GPIO_InitStructure.GPIO_Pin = BAT_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //Analog input
	GPIO_Init(BAT_GPIO_PORT, &GPIO_InitStructure); //Initialize GPIOC.4

	ADC_DeInit(BAT_ADC);												//Reset BAT_ADC, reset all registers of peripheral BAT_ADC to default values
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//ADC independent mode
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//Single channel mode
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//Single conversion mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //Conversion started by software, not by external trigger
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC data right aligned
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//Number of ADC channels in the regular conversion sequence
	ADC_Init(BAT_ADC, &ADC_InitStructure);								//Initialize peripheral ADCx with the specified parameters
	ADC_Cmd(BAT_ADC, ENABLE);											//Enable the specified BAT_ADC
	ADC_ResetCalibration(BAT_ADC);										//Start reset calibration
	while (ADC_GetResetCalibrationStatus(BAT_ADC))
		;						   //Wait for reset calibration to finish
	ADC_StartCalibration(BAT_ADC); //Start AD calibration
	while (ADC_GetCalibrationStatus(BAT_ADC))
		; //Wait for calibration to finish
}


// Get ADC value, ch: channel value 0~3
static uint16_t Adc_Get(uint8_t ch)
{
	uint16_t timeout = 1000;
	//Configure the regular group channel of the specified ADC, set its conversion order and sampling time
	ADC_RegularChannelConfig(BAT_ADC, ch, 1, ADC_SampleTime_239Cycles5);
	//Channel 1, regular sampling order value 1, sampling time 239.5 cycles
	ADC_SoftwareStartConvCmd(BAT_ADC, ENABLE); //Enable the software conversion function
	while (!ADC_GetFlagStatus(BAT_ADC, ADC_FLAG_EOC) && timeout--)
		;									//Wait for conversion to finish
	return ADC_GetConversionValue(BAT_ADC); //Return the latest BAT_ADC regular group conversion result
}


// Get the average of several ADC measurements, ch: channel value ; times: number of measurements
uint16_t Adc_Get_Average(uint8_t ch, uint8_t times)
{
	uint16_t temp_val = 0;
	uint8_t t;
	for (t = 0; t < times; t++)
	{
		temp_val += Adc_Get(ch);
	}
	if (times == 4)
	{
		temp_val = temp_val >> 2;
	}
	else
	{
		temp_val = temp_val / times;
	}
	return temp_val;
}


// Get the raw measured voltage value
float Adc_Get_Measure_Volotage(void)
{
	uint16_t adcx;
	float temp;
	adcx = Adc_Get_Average(BAT_ADC_CH, 4);
	temp = (float)adcx * (3.30f / 4096);
	return temp;
}


// Get the actual battery voltage before the voltage divider
float Adc_Get_Battery_Volotage(void)
{
	float temp;
	temp = Adc_Get_Measure_Volotage();
	// The actually measured value is slightly lower than the calculated one.
	temp = temp * 4.03f;    //temp*(10+3.3)/3.3; 
	return temp;
}


