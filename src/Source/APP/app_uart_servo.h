#ifndef __APP_UART_SERVO_H__
#define __APP_UART_SERVO_H__

#include "stm32f10x.h"

#define FLAG_RECV            1
#define FLAG_RXBUFF          2

#define MEDIAN_VALUE         2000

#define MID_ID5_MAX          3700
#define MID_ID5_MIN          380
// (uint16_t)((MID_ID5_MAX-MID_ID5_MIN)/3+MID_ID5_MIN)
#define MID_VAL_ID5          1486

#define MID_VAL_ID6          3100


#define ARM_MAX_VALUE        (4000)
#define ARM_MIN_VALUE        (96)


#define RX_MAX_BUF           8
#define MAX_SERVO_NUM        6

#define ARM_READ_TO_UART     0
#define ARM_READ_TO_FLASH    1
#define ARM_READ_TO_ARM      2


#define FLAG_OFFSET_ERROR    0
#define FLAG_OFFSET_OK       1
#define FLAG_OFFSET_OVER     2


/* Control the bus servo */
void UartServo_Ctrl(uint8_t id, uint16_t value, uint16_t time);

/* Set the buffer values for the synchronous write */
void UartServo_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6);

/* Write different parameters to several servos at the same time */
void UartServo_Sync_Write(uint16_t sync_time);

/* Set the bus servo torque switch, 0 to disable, 1 to enable*/
void UartServo_Set_Torque(uint8_t enable);

/* Write the target ID (1~250) */
void UartServo_Set_ID(uint8_t id);

/* Query the servo */
void UartServo_PING(uint8_t id);
/* Read the servo current angle */
void UartServo_Get_Angle(uint8_t id);

/* Receive data */
void UartServo_Revice(uint8_t Rx_Temp);

/* Read the bus servo flag */
uint8_t UartServo_Get_Flag(uint8_t flag);

/* Clear the bus servo flag */
void UartServo_Clear_Flag(uint8_t flag);

/* Parse the serial port data and send it to the serial port */
uint8_t UartServo_Rx_Parse(uint8_t request_id);

void UartServo_Send_ARM_Angle(uint8_t id, uint16_t value);
void UartServo_Send_ARM_Angle_Array(void);
void UartServo_Clear_Arm_Read_Vlaue(void);

void UartServo_Set_Median_Value(uint8_t id, uint16_t value);
int16_t UartServo_Get_Median_Offset(uint8_t id);
void UartServo_Read_All_Median_Value(void);
void UartServo_Init_Offset(void);
uint8_t UartServo_Offset_state(void);
void UartServo_Offset_Reset(void);

void UartServo_Set_Read_State(uint16_t state);
uint16_t UartServo_Get_Read_State(void);
void UartServo_Verify_Offset(uint8_t id, uint16_t value);
void UartServo_Send_Offset_State(uint8_t id, uint8_t state);

uint16_t UartServo_Test_Read_State(void);



#endif /* __APP_UART_SERVO_H__ */
