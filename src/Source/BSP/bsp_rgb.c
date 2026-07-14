#include "bsp_rgb.h"


#if RGB_DRV_SPI
#define TIMING_ONE           0xF8
#define TIMING_ZERO          0x80
#else
#define TIMING_ONE           61
#define TIMING_ZERO          29
#endif
#define BIT_LEN              24
#define BUFF_SIZE            (MAX_RGB * BIT_LEN + 2)


// Stores the color value of each LED, range: [0, 0xffffff].
uint32_t led_buf[MAX_RGB] = {0};
uint8_t RGB_Byte_Buffer[BUFF_SIZE] = {0};

// GPIO initialization
static void RGB_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(Colorful_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = Colorful_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Colorful_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(Colorful_PORT, Colorful_PIN);
}

// Initialize the DMA channel of the RGB LED strip.
static void RGB_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
#if RGB_DRV_SPI
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    DMA_DeInit(DMA2_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&SPI3->DR);           // Peripheral address: SPIx DR
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RGB_Byte_Buffer;           // Address of the data to send
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          // Transfer direction, from memory to register
    DMA_InitStructure.DMA_BufferSize = 0;                                       // Length of data to send, can be 0 at init, set when sending
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // Peripheral address is not incremented
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Memory address is auto-incremented by 1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     // Peripheral data width
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             // Memory data width
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // Transfer mode, send only once
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         // DMA transfer priority is high
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // Disable memory-to-memory
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);
#else
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM1->CCR1);         // Peripheral address: TIMx CCRx
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RGB_Byte_Buffer;           // Address of the data to send
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          // Transfer direction, from memory to register
    DMA_InitStructure.DMA_BufferSize = 0;                                       // Length of data to send, can be 0 at init, set when sending
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // Peripheral address is not incremented
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Memory address is auto-incremented by 1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // Peripheral data width
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             // Memory data width
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // Transfer mode, send only once
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         // DMA transfer priority is high
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // Disable memory-to-memory
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    /* TIM1 CC1 DMA Request enable */
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
#endif
}

#if RGB_DRV_SPI
// Initialize the SPI device of the RGB LED strip
static void RGB_Spi_Init(void)
{
    SPI_InitTypeDef SPIInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    SPIInitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPIInitStructure.SPI_Mode = SPI_Mode_Master;
    SPIInitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPIInitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPIInitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPIInitStructure.SPI_NSS = SPI_NSS_Soft;
    SPIInitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPIInitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPIInitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI3, &SPIInitStructure);
    SPI_Cmd(SPI3, ENABLE);
    SPI_CalculateCRC(SPI3, DISABLE);
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
}
#else
// Initialize the timer PWM output of the RGB LED strip.
static void RGB_Timer_Pwm_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //Reset the Timer back to its default values
    TIM_DeInit(TIM1);
    //Prescaler factor is 0, i.e. no prescaling, so the TIMER frequency is 72MHzre.TIM_Prescaler =0;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    //Set the counter overflow value: an update event occurs every 90 counts, 72000000/90=800k
    TIM_TimeBaseStructure.TIM_Period = 90 - 1;
    //Set the clock division
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //Set the counter mode to up-counting mode
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //Repetition counter not used, but must not be omitted
    //Apply the configuration to TIM1
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    //Set the default values
    TIM_OCStructInit(&TIM_OCInitStructure);

    //TIM1 CH1 output
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //Select whether it is PWM mode or compare mode
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Compare output enable, enable PWM output to the pin
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //Set whether the polarity is high or low
    //Set the duty cycle, duty cycle=(CCRx/ARR)*100% or (TIM_Pulse/TIM_Period)*100%
    TIM_OCInitStructure.TIM_Pulse = 30;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    //Enable the PWM output of TIM1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    //TIM_Cmd(TIM1,ENABLE);
    /* Timer 1 enables DMA*/
    TIM_SelectCCDMA(TIM1, ENABLE);
}
#endif

// RGB LED strip driver initialization
static void RGB_Driver_Init(void)
{
#if RGB_DRV_SPI
    RGB_Spi_Init();
#else
    RGB_Timer_Pwm_Init();
#endif
    RGB_DMA_Init();
}

// Initialize the LED strip
void RGB_Init(void)
{
    RGB_GPIO_Init();
    RGB_Driver_Init();
}

// Update the RGB LED strip
void RGB_Update(void)
{
    uint8_t i, j;
    for (j = 0; j < MAX_RGB; j++)
    {
        for (i = 0; i < BIT_LEN; i++)
        {
            RGB_Byte_Buffer[i + j * 24 + 1] = ((led_buf[j] >> (23 - i)) & 0x01) ? TIMING_ONE : TIMING_ZERO;
        }
    }
#if RGB_DRV_SPI
    DMA_SetCurrDataCounter(DMA2_Channel2, BUFF_SIZE); // Update the amount of data to transfer
    DMA_Cmd(DMA2_Channel2, ENABLE);                   // Enable the DMA channel, start the data transfer
    while (!DMA_GetFlagStatus(DMA2_FLAG_TC2))         // Wait for the transfer to complete
        ;
    DMA_Cmd(DMA2_Channel2, DISABLE); // Disable the DMA channel
    DMA_ClearFlag(DMA2_FLAG_TC2);    // Clear the DMA channel status
#else
    DMA_SetCurrDataCounter(DMA1_Channel5, BUFF_SIZE); // Update the amount of data to transfer
    DMA_Cmd(DMA1_Channel5, ENABLE);                   // Enable DMA channel 5, start the data transfer
    TIM_Cmd(TIM1, ENABLE);                            // Enable timer 1
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC5))         // Wait for the transfer to complete
        ;
    TIM_Cmd(TIM1, DISABLE); // Disable timer 1 and clear the timer parameters
    TIM1->CCR1 = 0;
    TIM1->CNT = 0;
    TIM1->SR = 0;
    DMA_Cmd(DMA1_Channel5, DISABLE); // Disable DMA channel 5
    DMA_ClearFlag(DMA1_FLAG_TC5);    // Clear the DMA channel status
#endif
}

// Set the color, index=[0, MAX_RGB] controls the matching LED color, index=0xFF controls all the LEDs.
void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = g << 16 | r << 8 | b;
    if (index < MAX_RGB)
    {
        led_buf[index] = color;
        return;
    }
    if (index == RGB_CTRL_ALL)
    {
        for (uint8_t i = 0; i < MAX_RGB; i++)
        {
            led_buf[i] = color;
        }
    }
}

// Set the RGB strip color value, index=[0, 16] controls the matching LED color, index=255 controls all the LEDs. color=0xggrrbb
void RGB_Set_Color_U32(uint8_t index, uint32_t color)
{
    if (index < MAX_RGB)
    {
        led_buf[index] = color;
        return;
    }
    if (index == RGB_CTRL_ALL)
    {
        for (uint8_t i = 0; i < MAX_RGB; i++)
        {
            led_buf[i] = color;
        }
    }
}

// Clear the color (turn the LEDs off)
void RGB_Clear(void)
{
    for (uint8_t i = 0; i < MAX_RGB; i++)
    {
        led_buf[i] = 0x0;
    }
}
