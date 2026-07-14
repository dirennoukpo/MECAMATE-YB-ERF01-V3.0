#include "bsp_key.h"
#include "bsp.h"

uint16_t g_key1_long_press = 0;

// Check if the key is pressed: returns KEY_PRESS when pressed, KEY_RELEASE when released
static uint8_t Key1_is_Press(void)
{
	if (!GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
	{
		return KEY_PRESS; // If the key is pressed, return KEY_PRESS
	}
	return KEY_RELEASE;   // If the key is in the released state, return KEY_RELEASE
}


// Blocking key scan: returns 1 when pressed, 0 when released
uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	/*Check whether a key is pressed */
	if (!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))
	{
		/*Wait for the key to be released */
		while (!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))
			;
		return KEY_PRESS;
	}
	else
		return KEY_RELEASE;
}

// Read the long-press state of key K1: returns 1 when the accumulated long-press time is reached, 0 otherwise.
// timeout is the configured duration, in seconds
uint8_t Key1_Long_Press(uint16_t timeout)
{
	if (g_key1_long_press > 0)
	{
		if (g_key1_long_press < timeout * 100 + 2)
		{
			g_key1_long_press++;
			if (g_key1_long_press == timeout * 100 + 2)
			{
				DEBUG("key2 long press\n");
				return 1;
			}
			return 0;
		}
	}
	return 0;
}

// Read the state of key K1: returns 1 when pressed, 0 when released.
// mode: sets the mode, 0: keeps returning 1 while pressed; 1: returns 1 only once per press
uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
		g_key1_long_press = 0;
	}
	if (key1_state == 2)
	{
		g_key1_long_press = 1;
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}

// Initialize the key
void Key_GPIO_Init(void)
{
	/*Define a structure of type GPIO_InitTypeDef*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*Enable the clock of the key port*/
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK, ENABLE);
	//Select the pin of the key
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN;
	//Set the key pin as floating input
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//Initialize the key using the structure
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
}


/*********************************************END OF FILE**********************/
