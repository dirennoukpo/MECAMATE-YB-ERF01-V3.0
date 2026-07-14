#ifndef __Flash_H__
#define __Flash_H__
#include "stm32f10x.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
//Set according to your own needs
#define STM32_FLASH_SIZE      256      //FLASH capacity of the selected STM32 (unit: K)
#define STM32_FLASH_WREN      1        //Enable FLASH write (0, disable; 1, enable)

// Because high-density devices define each sector as 2K, while low- and medium-density devices define it as 1K
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024 //bytes
#else
#define STM_SECTOR_SIZE 2048
#endif


//FLASH start address
#define STM32_FLASH_BASE      0x08000000      //STM32 FLASH start address
#define STM32_FLASH_END       0x0803FFFF      //STM32F1RCT6 FLASH end address


//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t Flash_ErasePage(uint32_t paddr);                                    //Erase page

void Flash_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);    //Write data of specified length starting from the specified address
void Flash_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);       //Read data of specified length starting from the specified address

#endif
