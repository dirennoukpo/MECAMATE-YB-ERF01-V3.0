#include "bsp_io_i2c.h"
#include "bsp.h"


//IO direction setting
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)3<<12;}

//IO operation functions	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //Input SDA 

// Pin wait time, can be modified according to the chip clock, as long as it meets the communication requirements.
#define DELAY_FOR_COUNT      10


static void Delay_For_Pin(vu8 nCount)
{
    // volatile: without it, GCC removes the inner loop (empty body, non-volatile
    // counter) and the delay drops from ~240 to ~12 cycles -> SCL > 1 MHz.
    // ARMCC did emit the loop. This is the only addition to the original file.
    volatile uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < DELAY_FOR_COUNT; i++);
    }
}


/**
 * @Brief: Initialize the interface pins used by I2C
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Init(void)
{
    RCC->APB2ENR |= 1 << 3;   //First enable the peripheral IO PORTB clock
    GPIOB->CRH &= 0XFFFF00FF; //PB 10/11 push-pull output
    GPIOB->CRH |= 0X00003300;
}

/**
 * @Brief: Generate the IIC start signal
 * @Note:
 * @Parm: void
 * @Retval: void
 */
int IIC_Start(void)
{
    SDA_OUT(); //sda line output
    IIC_SDA = 1;
    if (!READ_SDA)
        return 0;
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    if (READ_SDA)
        return 0;
    Delay_For_Pin(2);
    IIC_SCL = 0; //Clamp the I2C bus, ready to send or receive data
    return 1;
}

/**
 * @Brief: Generate the IIC stop signal
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Stop(void)
{
    SDA_OUT(); //sda line output
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    Delay_For_Pin(2);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SDA = 1; //Send the I2C bus stop signal
    Delay_For_Pin(2);
}

/**
 * @Brief: Wait for the acknowledge signal to arrive
 * @Note:
 * @Parm:
 * @Retval: 1:acknowledge received OK | 0:acknowledge reception failed
 */
int IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); //SDA set as input
    IIC_SDA = 1;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            IIC_Stop();
            return 0;
        }
        Delay_For_Pin(1);
    }
    IIC_SCL = 0; //Clock output 0
    return 1;
}

/**
 * @Brief: Generate an ACK acknowledge
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SCL = 0;
}

/**
 * @Brief: Generate a NACK acknowledge
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SCL = 0;
}

/**
 * @Brief: IIC send one byte
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL = 0; //Pull the clock low to start data transfer
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        Delay_For_Pin(1);
        IIC_SCL = 1;
        Delay_For_Pin(1);
        IIC_SCL = 0;
        Delay_For_Pin(1);
    }
}

uint8_t g_addr[128] = {0};


void i2c_scanf_addr(void)
{
    uint8_t i2c_count = 0;
    for (int i = 1; i < 128; i++)
    {
        if (!IIC_Start())
        {
            printf("External IIC Start Error:%d\n", i);
            return;
        }
        IIC_Send_Byte(i << 1);
        if (!IIC_Wait_Ack())
        {
            if (i == 127)
            {
                printf("External IIC Scanf End, Count=%d\n", i2c_count);
            }
            continue;
        }
        IIC_Stop();
        i2c_count++;
        printf("External IIC Found address:0x%2x\n", i);
        g_addr[i] = i;
    }
}


/**
 * @Brief: I2C data write function
 * @Note:
 * @Parm: addr:I2C address | reg:register | len:data length | data:data pointer
 * @Retval: 0:stop | 1:continuous write
 */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack())
        {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}

/**
 * @Brief: I2C read function
 * @Note:
 * @Parm: parameters similar to the write function
 * @Retval:
 */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1) + 1);
    IIC_Wait_Ack();
    while (len)
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**
 * @Brief: Read 1 byte; when ack=1 send ACK, when ack=0 send nACK
 * @Note:
 * @Parm:
 * @Retval:
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); //SDA set as input
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        Delay_For_Pin(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        Delay_For_Pin(2);
    }
    if (ack)
        IIC_Ack(); //Send ACK
    else
        IIC_NAck(); //Send nACK
    return receive;
}

/**
 * @Brief: Read one value from a given register of a given device
 * @Note: 
 * @Parm: I2C_Addr  target device address | addr     register address
 * @Retval: 
 */
//This function is unusable, there is a problem to investigate!!!!
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;

    IIC_Start();
    IIC_Send_Byte(I2C_Addr); //Send write command
    res++;
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    res++; //Send address
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(I2C_Addr + 1);
    res++; //Enter receive mode
    IIC_Wait_Ack();
    res = IIC_Read_Byte(0);
    IIC_Stop(); //Generate a stop condition

    return res;
}

/**
 * @Brief: Read length values from a given register of a given device
 * @Note: 
 * @Parm: dev  target device address | reg   register address | length number of bytes to read | *data  pointer where the read data will be stored
 * @Retval: number of bytes read
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;

    IIC_Start();
    IIC_Send_Byte(dev); //Send write command
    IIC_Wait_Ack();
    IIC_Send_Byte(reg); //Send address
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(dev + 1); //Enter receive mode
    IIC_Wait_Ack();

    for (count = 0; count < length; count++)
    {

        if (count != length - 1)
            data[count] = IIC_Read_Byte(1); //Read data with ACK
        else
            data[count] = IIC_Read_Byte(0); //NACK on the last byte
    }
    IIC_Stop(); //Generate a stop condition
    return count;
}

/**
 * @Brief: Write several bytes to a given register of a given device
 * @Note: 
 * @Parm: dev  target device address | reg   register address | length number of bytes to write | *data  start address of the data to be written
 * @Retval: returns whether it succeeded
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{

    uint8_t count = 0;
    IIC_Start();
    IIC_Send_Byte(dev); //Send write command
    IIC_Wait_Ack();
    IIC_Send_Byte(reg); //Send address
    IIC_Wait_Ack();
    for (count = 0; count < length; count++)
    {
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
    }
    IIC_Stop(); //Generate a stop condition

    return 1; //status == 0;
}

/**
 * @Brief: Read one value from a given register of a given device
 * @Note: 
 * @Parm: dev  target device address | reg    register address | *data  address where the read data will be stored
 * @Retval: 1
 */
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    *data = I2C_ReadOneByte(dev, reg);
    return 1;
}

/**
 * @Brief: Write one byte to a given register of a given device
 * @Note: 
 * @Parm: dev  target device address | reg    register address | data  byte to be written
 * @Retval: 1
 */
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**
 * @Brief: Read Modify Write several bits inside one byte of a given register of a given device
 * @Note: 
 * @Parm: dev  target device address | reg    register address | bitStart  start bit of the target byte | length   bit length | data    holds the value of the target byte bits to change
 * @Retval: 1:success | 0:failure
 */
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0)
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    else
    {
        return 0;
    }
}

/**
 * @Brief: Read Modify Write 1 bit inside one byte of a given register of a given device
 * @Note: 
 * @Parm: dev  target device address | reg    register address | bitNum  bitNum bit of the target byte to modify | data  when 0, the target bit is cleared to 0, otherwise it is set
 * @Retval: 1:success | 0:failure
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}
