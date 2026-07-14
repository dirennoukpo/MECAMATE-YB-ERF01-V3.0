#include "bsp_usart.h"
#include "bsp.h"
#include "stdio.h"
#include "stdarg.h"
#include "protocol.h"
#include "app_uart_servo.h"
#include "app_sbus.h"

#include "app.h"

#define ENABLE_USART1_DMA       1


#if ENABLE_USART1_DMA

#define MAX_LEN                 100
#define USART1_DR_Address       0x40013804

uint8_t g_dma_buff[MAX_LEN] = {0};

static void USART1_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channel4 (triggered by USART1 Tx event) Config */
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);     // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_dma_buff;            // Tx memory address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // peripheral is the transfer destination, i.e. send data
    DMA_InitStructure.DMA_BufferSize = 0;                                   // transfer length is 0

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // Peripheral address not incremented
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // Memory address auto-increment by 1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Peripheral data width 8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // Memory data width 8bit
	DMA_InitStructure.DMA_Mode =  DMA_Mode_Normal;                          // DMA_Mode_Normal(transfer once), DMA_Mode_Circular (transfer continuously)
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     // (DMA transfer priority is high)
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // not memory-to-memory
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);                            // 初始化
	// USART_ITConfig(USART1, USART_IT_TC, ENABLE);                            // Enable UART transmit-complete interrupt; the interrupt must be configured, otherwise it affects startup
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);                          // Enable DMA UART transmit request
}

#endif



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @Brief: Initialize IO USART1
 * @Note: 
 * @Parm: baudrate: baud rate
 * @Retval: 
 */
void USART1_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	// Enable the clock of the UART GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Enable the clock of the UART peripheral
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Configure the USART Tx GPIO as alternate-function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure the USART Rx GPIO as floating input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Usart NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //preemption priority 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //sub-priority 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ channel enable
	NVIC_Init(&NVIC_InitStructure);							  //initialize the VIC register with the specified parameters

	// Configure the UART working parameters
	// Configure the baud rate
	USART_InitStructure.USART_BaudRate = baudrate;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// Configure the stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(USART1, &USART_InitStructure);
	//开启串口接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	// 使能串口
	USART_Cmd(USART1, ENABLE);

	// 完成TC状态位的清除,防止硬件复位重启之后, 第一个字节丢失
	USART_ClearFlag(USART1, USART_FLAG_TC);

	#if ENABLE_USART1_DMA
	USART1_DMA_Init();
	#endif
}

/**
 * @Brief: UART1发送数据
 * @Note: 
 * @Parm: ch:待发送的数据 
 * @Retval: 
 */
void USART1_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

/**
 * @Brief: UART1发送数据
 * @Note: 
 * @Parm: BufferPtr:待发送的数据  Length:数据长度
 * @Retval: 
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	if (!Length) return;  // 如果长度不符合则返回
	#if ENABLE_USART1_DMA
	while (DMA_GetCurrDataCounter(DMA1_Channel4))
        ; // 检查DMA发送通道内是否还有数据
    if (BufferPtr)
        memcpy(g_dma_buff, BufferPtr, (Length > MAX_LEN ? MAX_LEN : Length));
    // DMA发送数据-要先关 设置发送长度 开启DMA
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = Length;    // 设置发送长度
    DMA_Cmd(DMA1_Channel4, ENABLE);   // 启动DMA发送

	#else
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
	#endif
}

/**
 * @Brief: 串口1中断函数 
 * @Note: 
 * @Parm: 
 * @Retval: 
 */
void USART1_IRQHandler(void)
{
	uint8_t Rx1_Temp = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		// Rx1_Temp = USART1->DR; //(USART1->DR)/(USART_ReceiveData(USART1));  //读取接收到的数据
		Rx1_Temp = USART_ReceiveData(USART1);
		#if PID_ASSISTANT_EN
		protocol_data_recv(&Rx1_Temp, 1);
		#else
		Upper_Data_Receive(Rx1_Temp);
		#endif
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if ENABLE_USART2
/************************************************
函数名称 ： USART2_Init
功    能 ： UART初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void USART2_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable the clock of the UART GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Enable the clock of the UART peripheral
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// Configure the USART Tx GPIO as alternate-function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure the USART Rx GPIO as floating input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Usart2 NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ channel enable
	NVIC_Init(&NVIC_InitStructure);							  //initialize the NVIC register with the specified parameters

	// Configure the UART working parameters
	#if ENABLE_SBUS
	// Configure the baud rate
	USART_InitStructure.USART_BaudRate = baudrate;
	// Configure the data word length: 9-bit data format, including 1 parity bit and 8 data bits
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	// Configure 2 stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	// 配置校验位:偶校验位USART_Parity_Even
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	// 配置硬件流控制:无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	#else
	// Configure the baud rate
	USART_InitStructure.USART_BaudRate = baudrate;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// Configure the stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	#endif
	// 完成串口的初始化配置
	USART_Init(USART2, &USART_InitStructure);

	//开启串口接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	// 使能串口
	USART_Cmd(USART2, ENABLE);
}

/************************************************
函数名称 ： USART2_Send_U8
功    能 ： UART2发送一个字符
参    数 ： Data --- 数据
返 回 值 ： 无
*************************************************/
void USART2_Send_U8(uint8_t Data)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART2, Data);
}

/************************************************
函数名称 ： USART2_Send_ArrayU8
功    能 ： 串口1发送N个字符
参    数 ： pData ---- 字符串
            Length --- 长度
返 回 值 ： 无
*************************************************/
void USART2_Send_ArrayU8(uint8_t *pData, uint16_t Length)
{
	while (Length--)
	{
		USART2_Send_U8(*pData);
		pData++;
	}
}

// 串口连接蓝牙
void USART2_Connect_BT(void)
{
	#if ENABLE_SBUS == 0
	uint8_t cmd_bt[] = {0x41, 0x54, 0x2B, 0x43, 0x4F, 0x4E, 0x41, 0x30, 0x78, 0x45, 
		0x43, 0x32, 0x34, 0x42, 0x38, 0x30, 0x36, 0x30, 0x30, 0x32, 0x45, 0x0D, 0x0A};
	USART2_Send_ArrayU8(cmd_bt, sizeof(cmd_bt));
	#endif
}

/*  串口中断接收处理 */
void USART2_IRQHandler(void)
{
	uint8_t Rx2_Temp = 0;
	// UART2_ClearITPendingBit(UART2_IT_RXNE);
	/* 读数据会清空中断位 */
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
		;
	Rx2_Temp = USART_ReceiveData(USART2);
	#if ENABLE_SBUS
	SBUS_Reveive(Rx2_Temp);
	#elif PID_ASSISTANT_EN
	protocol_data_recv(&Rx2_Temp, 1);
	#else
	Upper_Data_Receive(Rx2_Temp);
	#endif
}

#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if ENABLE_USART3
void USART3_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable the clock of the UART GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	// Enable the clock of the UART peripheral
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	// Configure the USART Tx GPIO as alternate-function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure the USART Rx GPIO as floating input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//Usart NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //preemption priority 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		  //sub-priority 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ channel enable
	NVIC_Init(&NVIC_InitStructure);							  //initialize the VIC register with the specified parameters

	// Configure the UART working parameters
	// Configure the baud rate
	USART_InitStructure.USART_BaudRate = baudrate;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// Configure the stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(USART3, &USART_InitStructure);

	// 开启串口接受中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  

	// 使能串口
	USART_Cmd(USART3, ENABLE);
}

/************************************************
函数名称 ： USART3_Send_U8
功    能 ： UART3发送一个字符
参    数 ： Data --- 数据
返 回 值 ： 无
*************************************************/
void USART3_Send_U8(uint8_t Data)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART3, Data);
}

/************************************************
函数名称 ： USART3_Send_ArrayU8
功    能 ： 串口发送N个字符
参    数 ： pData ---- 字符串
            Length --- 长度
返 回 值 ： 无
*************************************************/
void USART3_Send_ArrayU8(uint8_t *pData, uint16_t Length)
{
	while (Length--)
	{
		USART3_Send_U8(*pData);
		pData++;
	}
}

/*  串口中断接收处理 */
void USART3_IRQHandler(void)
{
    /* 读数据会清空中断位 */
    // while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
    //     ;
    // uint8_t Rx3_Temp = USART_ReceiveData(USART3);
	// UartServo_Revice(Rx3_Temp);
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		uint8_t Rx3_Temp = USART_ReceiveData(USART3);
		UartServo_Revice(Rx3_Temp);
	}
}
#endif

#if ENABLE_UART4
void UART4_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// Enable the clock of the UART GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	// Enable the clock of the UART peripheral
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// Configure the USART Tx GPIO as alternate-function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure the USART Rx GPIO as floating input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure the UART working parameters
	// Configure the baud rate
	USART_InitStructure.USART_BaudRate = baudrate;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// Configure the stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(UART4, &USART_InitStructure);

	// 使能串口
	USART_Cmd(UART4, ENABLE);
	USART_ClearFlag(UART4, USART_FLAG_TC);
}
#endif

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
	USART_SendData(DEBUG_USARTx, (uint8_t)ch);

	/* 等待发送完毕 */
	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
		;
	return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	/* 等待串口输入数据 */
	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET)
		;
	return (int)USART_ReceiveData(DEBUG_USARTx);
}
