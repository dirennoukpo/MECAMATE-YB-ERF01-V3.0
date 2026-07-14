#ifndef __MPUMPU_IIC_H
#define __MPUMPU_IIC_H
#include "bsp.h"

#define ENABLE_I2C_PB10_PB11      0

#if ENABLE_I2C_PB10_PB11
#define MPU_SDA_IN()  	{GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() 	{GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO operation functions
#define MPU_IIC_SCL    PBout(10) //SCL
#define MPU_IIC_SDA    PBout(11) //SDA	 
#define MPU_READ_SDA   PBin(11)  //SDA input 

#else
///*PB13 PB15 OK*/
//IO direction settings
#define MPU_SDA_IN()  	{GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)8<<28;}
#define MPU_SDA_OUT() 	{GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)3<<28;}

//IO operation functions	 
#define MPU_IIC_SCL    PBout(13) //SCL
#define MPU_IIC_SDA    PBout(15) //SDA	 
#define MPU_READ_SDA   PBin(15)  //SDA input 

#endif

//All IIC operation functions
void MPU_IIC_Delay(void);				//MPU IIC delay function
void MPU_IIC_Init(void);                //Initialize IIC IO pins				 
void MPU_IIC_Start(void);				//Send IIC start signal
void MPU_IIC_Stop(void);	  			//Send IIC stop signal
void MPU_IIC_Send_Byte(u8 txd);			//IIC send one byte
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC read one byte
u8 MPU_IIC_Wait_Ack(void); 				//IIC wait for ACK signal
void MPU_IIC_Ack(void);					//IIC send ACK signal
void MPU_IIC_NAck(void);				//IIC does not send ACK signal

uint8_t MPU_Write_Byte(uint8_t devaddr,uint8_t reg,uint8_t data);
uint8_t MPU_Read_Byte(uint8_t devaddr,uint8_t reg);
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

void MPU_Scanf_Addr(void);

#endif
















