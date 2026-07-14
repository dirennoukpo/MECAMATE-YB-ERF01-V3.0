#include "bsp_mpuiic.h"

static void Delay_For_Pin(vu8 nCount)
{
    // volatile: without it, GCC removes the inner loop (empty body, non-volatile
    // counter) and the delay drops from ~240 to ~12 cycles -> SCL > 1 MHz.
    // ARMCC did emit the loop. Only addition to the original file.
    volatile uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < 10; i++);
    }
}

#define delay_us  Delay_For_Pin


//Initialize IIC
void MPU_IIC_Init(void)
{		
	#if ENABLE_I2C_PB10_PB11
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//First enable the peripheral IO PORTC clock 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	 // Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //Push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO port speed is 50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //Initialize GPIO according to the configured parameters 	
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
	#else			     
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//First enable the peripheral IO PORTC clock 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;	 // Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //Push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO port speed is 50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //Initialize GPIO according to the configured parameters 	
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_15);
	#endif
}

//Generate IIC start signal
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda line output
	MPU_IIC_SDA=1;	  	  
	MPU_IIC_SCL=1;
	delay_us(4);
 	MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	MPU_IIC_SCL=0;//Clamp the I2C bus, prepare to send or receive data 
}	  
//Generate IIC stop signal
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda line output
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	MPU_IIC_SCL=1;  
	MPU_IIC_SDA=1;//Send I2C bus end signal
	delay_us(4);							   	
}
//Wait for the acknowledge signal
//Return value: 1, acknowledge reception failed
//        0, acknowledge reception succeeded
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA set as input  
	MPU_IIC_SDA=1;delay_us(1);   
	MPU_IIC_SCL=1;delay_us(1);	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;//Clock output 0 	   
	return 0;  
} 
//Generate ACK acknowledge
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=0;
	delay_us(2);
	MPU_IIC_SCL=1;
	delay_us(2);
	MPU_IIC_SCL=0;
}
//Do not generate ACK acknowledge		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	delay_us(2);
	MPU_IIC_SCL=1;
	delay_us(2);
	MPU_IIC_SCL=0;
}					 				     
//IIC send one byte
//Returns whether the slave acknowledged
//1, acknowledged
//0, no acknowledge			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;//Pull the clock low to start data transmission
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);
		MPU_IIC_SCL=1;
		delay_us(2); 
		MPU_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//Read 1 byte; when ack=1 send ACK, when ack=0 send nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA set as input
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL=0; 
        delay_us(2); 
		MPU_IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//Send nACK
    else
        MPU_IIC_Ack(); //Send ACK   
    return receive;
}


// IIC continuous write, return value:0,normal , other,error code
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //Send device address + write command
	if (MPU_IIC_Wait_Ack())				//Wait for acknowledge
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg); //Write register address
	MPU_IIC_Wait_Ack();		//Wait for acknowledge
	for (i = 0; i < len; i++)
	{
		MPU_IIC_Send_Byte(buf[i]); //Send data
		if (MPU_IIC_Wait_Ack())	   //Wait for ACK
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC continuous read, return value:0,normal , other,error code
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //Send device address + write command
	if (MPU_IIC_Wait_Ack())				//Wait for acknowledge
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg); //Write register address
	MPU_IIC_Wait_Ack();		//Wait for acknowledge
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1); //Send device address + read command
	MPU_IIC_Wait_Ack();					//Wait for acknowledge
	while (len)
	{
		if (len == 1)
			*buf = MPU_IIC_Read_Byte(0); //Read data, send nACK
		else
			*buf = MPU_IIC_Read_Byte(1); //Read data, send ACK
		len--;
		buf++;
	}
	MPU_IIC_Stop(); //Generate a stop condition
	return 0;
}

// IIC write one byte, return value:0,normal , other,error code
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //Send device address + write command
	if (MPU_IIC_Wait_Ack())				//Wait for acknowledge
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);	 //Write register address
	MPU_IIC_Wait_Ack();		 //Wait for acknowledge
	MPU_IIC_Send_Byte(data); //Send data
	if (MPU_IIC_Wait_Ack())	 //Wait for ACK
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC read one byte, return value:0,normal , other,error code
uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg)
{
	uint8_t res;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0); //Send device address + write command
	MPU_IIC_Wait_Ack();					//Wait for acknowledge
	MPU_IIC_Send_Byte(reg);				//Write register address
	MPU_IIC_Wait_Ack();					//Wait for acknowledge
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1); //Send device address + read command
	MPU_IIC_Wait_Ack();					//Wait for acknowledge
	res = MPU_IIC_Read_Byte(0);			//Read data, send nACK
	MPU_IIC_Stop();						//Generate a stop condition
	return res;
}

void MPU_Scanf_Addr(void)
{
    uint8_t i2c_count = 0;
	MPU_IIC_Init();
	MPU_ADDR_CTRL();
    for (int i = 1; i < 128; i++)
    {
        MPU_IIC_Start();
        MPU_IIC_Send_Byte(i << 1);
        if (MPU_IIC_Wait_Ack())
        {
            if (i == 127)
            {
                printf("MPU IIC Scanf End, Count=%d\n", i2c_count);
            }
            continue;
        }
        MPU_IIC_Stop();
        i2c_count++;
        printf("MPU IIC Found address:0x%2x\n", i);
    }
}
