#ifndef __BSP_H__
#define __BSP_H__

#include "stdio.h"
#include "stdint.h"

#include "stm32f10x.h"
#include "bsp_common.h"

#include "yb_debug.h"
#include "bsp_beep.h"
#include "bsp_timer.h"
#include "bsp_wdg.h"
#include "bsp_usart.h"
#include "bsp_io_i2c.h"
#include "bsp_motor.h"
#include "bsp_key.h"
#include "bsp_encoder.h"
#include "bsp_pwmServo.h"
#include "bsp_ssd1306.h"
#include "bsp_spi.h"
#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_rgb.h"
#include "bsp_mpu9250.h"



#define VERSION_MAJOR          0x03
#define VERSION_MINOR          0x05
#define VERSION_PATCH          0x01


//JTAG mode setting definitions
#define JTAG_SWD_DISABLE       0X02
#define SWD_ENABLE             0X01
#define JTAG_SWD_ENABLE        0X00


// Mode definitions
#define MODE_STANDARD          (0)
#define MODE_TEST              (1)


// IMU type definitions
typedef enum 
{
    IMU_TYPE_ICM20948 = 0,
    IMU_TYPE_MPU9250 = 1,

    IMU_TYPE_MAX = 0xFF
} IMU_Type_t;


void delay_init(void);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);


void Bsp_Init(void);

void Bsp_JTAG_Set(uint8_t mode);

void Bsp_Send_Version(void);

void Bsp_Led_Show_State(void);
void Bsp_Led_Show_Low_Battery(uint8_t enable_beep);
void Bsp_Led_Show_Overvoltage_Battery(uint8_t enable_beep);

void Bsp_Set_TestMode(uint16_t mode);
uint8_t Bsp_Get_TestMode(void);

uint8_t Bsp_Get_Imu_Type(void);
void Bsp_Imu_Type_None(void);

void Bsp_Long_Beep_Alarm(void);

void Bsp_Reset_MCU(void);


/* ---------------------------------------------------------------------------
 * Boot progress, shown on the OLED as a large percentage (0% -> 100%).
 *
 * The board takes about 11.5 s to boot, and 95% of that is the ICM20948 init:
 * ~2.2 s loading the DMP firmware over SPI2 (14250 bytes written, then read back
 * to verify), then ~8.6 s of self-test, of which 8.2 s is pure sleeping.
 * Measured on the board, not guessed.
 *
 * A milestone-based percentage would sit frozen at one value for 8.6 s, which is
 * exactly the problem we are trying to fix. So the progress is driven by an
 * ESTIMATED ELAPSED TIME, fed from the two things that actually consume the boot:
 *   - Bsp_Boot_Progress_Bytes(): every byte moved over SPI2 by the InvenSense
 *     driver (~0.0765 ms per byte, measured);
 *   - Bsp_Boot_Progress_Sleep(): every millisecond the driver spends in inv_sleep().
 * Both hooks live in Source/MEMS/inv_mems_drv_hook.c.
 * ------------------------------------------------------------------------- */
void Bsp_Boot_Progress_Begin(void);
void Bsp_Boot_Progress_Bytes(uint32_t bytes);
void Bsp_Boot_Progress_Sleep(uint32_t ms);
void Bsp_Boot_Progress_End(void);

#endif
