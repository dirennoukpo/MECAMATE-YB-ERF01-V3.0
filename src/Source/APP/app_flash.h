#ifndef __APP_FLASH_H__
#define __APP_FLASH_H__

#include "stdint.h"
#include "bsp_flash.h"

/******************************Flash address configuration**********************************************/
// Sector used to save the data
#define FLASH_DATA_SECTOR     120             //STM32F103RCT6 has 128 sectors in total (0~127), each sector is 2K

// Set the FLASH save address (must be even, and its value must be greater than the FLASH size used by this code + 0X08000000)

// Factory test mode switch. Size is 2 bytes.
#define F_TEST_MODE_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x00)

// Switch to reset the Flash values. Size is 2 bytes.
#define F_RESET_ALL_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x02)

// Save address of the auto data report switch state, size is 2 bytes
#define F_CAR_TYPE_ADDR           (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x04)

// Save address of the auto data report switch state, size is 2 bytes
#define F_AUTO_REPORT_ADDR        (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x06)

// Save address of the gyroscope yaw angle adjustment state, size is 2 bytes
#define F_IMU_STATE_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x0A)

// RGB light effect, size is 2 bytes
#define F_RGB_EFFECT_ADDR         (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x10)

// YAW PID parameters, size is 6 bytes
#define F_PID_YAW_ADDR            (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x20)

// PID parameters, size is 24 bytes
#define F_PID_ADDR                (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x30)

// Robotic arm median offset parameters, size is 14 bytes
#define F_ARM_OFFSET_ADDR         (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x50)



#define F_AKM_ANGLE_ADDR          (STM32_FLASH_BASE + FLASH_DATA_SECTOR * STM_SECTOR_SIZE + 0x70)

/******************************Flash address configuration**********************************************/


/******************************Flash macro-defined variable configuration******************************************/
// If the value read from address F_RESET_ALL_ADDR is not FLASH_RESET_OK, all values are automatically restored to their defaults.
#define FLASH_RESET_OK               0xAA55


/******************************Flash macro-defined variable configuration******************************************/


void Flash_Init(void);
void Flash_Reset_All_Value(void);

void Flash_Set_CarType(uint8_t carType);

void Flash_Set_Auto_Report(uint8_t enable);

void Flash_Set_PID(uint8_t motor_id, float kp, float ki, float kd);

void Flash_Set_Yaw_PID(float kp, float ki, float kd);


void Flash_Reset_ARM_Median_Value(void);
void Flash_Set_ARM_Median_Value(uint8_t id, uint16_t value);
void Flash_Read_ARM_Median_Value(uint8_t id, uint16_t* value);

void Flash_Set_AKM_Angle(uint16_t angle);

void Flash_TestMode_Init(void);
void Flash_Set_TestMode(uint8_t mode);

#endif /* __APP_FLASH_H__ */
