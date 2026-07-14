#include "bsp_spi.h"
#include "bsp.h"

static void Spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //PORTB clock enable

    // CLK  MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);

    // MISO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //Pull-up input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// SPI initialization, SPI2, full duplex, software NSS
void SPI2_Init(void)
{
    Spi_gpio_init();
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2 clock enable
    //Set the SPI unidirectional or bidirectional data mode: SPI set to 2-line bidirectional full duplex
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    //Set the SPI operating mode: set as SPI master
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    //Set the SPI data size: SPI transmits/receives an 8-bit frame structure
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    //The idle state of the serial synchronous clock is high level
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    //Data is sampled on the second transition edge (rising or falling) of the serial synchronous clock
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    //NSS signal managed by hardware (NSS pin) or by software (using the SSI bit): internal NSS signal is controlled by the SSI bit
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //Define the baud rate prescaler value: baud rate prescaler value is 256
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    //Specify whether data transfer starts from the MSB or the LSB: data transfer starts from the MSB
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    //Polynomial for the CRC value calculation
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    //Enable the SPI peripheral
    SPI_Cmd(SPI2, ENABLE);
}


//SPIx read/write one byte
//TxData: the byte to be written
uint8_t SPI2_ReadWriteByte(uint8_t TxData, uint8_t* RxData)
{
    uint8_t retry = 0;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //Check whether the specified SPI flag is set: transmit buffer empty flag
    {
        retry++;
        if(retry > 200)return 1;
    }

    SPI_I2S_SendData(SPI2, TxData); //Send one data item through the SPIx peripheral
    retry = 0;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //Check whether the specified SPI flag is set: receive buffer not empty flag
    {
        retry++;

        if(retry > 200)return 1;
    }

	*RxData = SPI_I2S_ReceiveData(SPI2);
	
    return 0; 
}
