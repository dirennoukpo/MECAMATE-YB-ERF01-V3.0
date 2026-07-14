#include "bsp_flash.h"
#include "bsp.h"
#include "bsp_usart.h"

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)


uint16_t Flash_BUF[STM_SECTOR_SIZE / 2]; //at most 2K bytes


//Unlock the STM32 FLASH
static void Flash_Unlock(void)
{
	FLASH->KEYR = FLASH_KEY1; //Write the unlock sequence.
	FLASH->KEYR = FLASH_KEY2;
}

//Lock the flash
static void Flash_Lock(void)
{
	FLASH->CR |= 1 << 7; //Lock
}

//Get the FLASH status
static uint8_t Flash_GetStatus(void)
{
	uint32_t res;
	res = FLASH->SR;
	if (res & (1 << 0))
		return 1; //Busy
	else if (res & (1 << 2))
		return 2; //Programming error
	else if (res & (1 << 4))
		return 3; //Write protection error
	return 0;	  //Operation complete
}

//Wait for the operation to complete
//time: length of the delay
//Return value: status.
static uint8_t Flash_WaitDone(uint16_t time)
{
	uint8_t res;
	// volatile: otherwise GCC deletes the timing loop below (empty
	// body, non-volatile counter), as it did for Delay_For_Pin.
	// ARMCC emitted it. The timeout budget would lose a factor of ~9.
	volatile uint8_t i;
	do
	{
		res = Flash_GetStatus();
		if (res != 1)
			break; //Not busy, no need to wait, exit directly.
		for (i = 0; i < 10; i++); // Add a delay
		time--;
	} while (time);
	if (time == 0)
		res = 0xff; //TIMEOUT
	return res;
}


//Write a half word at the specified FLASH address
//faddr: specified address (this address must be a multiple of 2!!)
//dat: data to write
//Return value: write status
static uint8_t Flash_WriteHalfWord(uint32_t faddr, uint16_t dat)
{
	uint8_t res;
	res = Flash_WaitDone(0XFF);
	if (res == 0) //OK
	{
		FLASH->CR |= 1 << 0;		   //Programming enable
		*(vu16 *)faddr = dat;		   //Write the data
		res = Flash_WaitDone(0XFF); //Wait for the operation to complete
		if (res != 1)				   //Operation successful
		{
			FLASH->CR &= ~(1 << 0); //Clear the PG bit.
		}
	}
	return res;
}

//Read the half word (16-bit data) at the specified address
//faddr: read address
//Return value: the corresponding data.
static uint16_t Flash_ReadHalfWord(uint32_t faddr)
{
	return *(vu16 *)faddr;
}

#if STM32_FLASH_WREN //If write is enabled

//Write without check
//WriteAddr: start address
//pBuffer: data pointer
//NumToWrite: number of half words (16-bit)
static void Flash_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	uint16_t i;
	for (i = 0; i < NumToWrite; i++)
	{
		Flash_WriteHalfWord(WriteAddr, pBuffer[i]);
		WriteAddr += 2; //Address increases by 2.
	}
}

//Write data of a given length starting at the specified address
//WriteAddr: start address (this address must be a multiple of 2!!)
//pBuffer: data pointer
//NumToWrite: number of half words (16-bit) (i.e. the count of 16-bit data to write.)
void Flash_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	uint32_t secpos;	   //Sector address
	uint16_t secoff;	   //Offset address inside the sector (counted in 16-bit words)
	uint16_t secremain; //Remaining space inside the sector (counted in 16-bit words)
	uint16_t i;
	uint32_t offaddr; //Address with 0X08000000 removed
	if (WriteAddr < STM32_FLASH_BASE || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
		return;								  //Illegal address
	Flash_Unlock();						  //Unlock
	offaddr = WriteAddr - STM32_FLASH_BASE;	  //Actual offset address.
	secpos = offaddr / STM_SECTOR_SIZE;		  //Sector address  0~127 for STM32F103RCT6
	secoff = (offaddr % STM_SECTOR_SIZE) / 2; //Offset inside the sector (2 bytes as the base unit.)
	secremain = STM_SECTOR_SIZE / 2 - secoff; //Size of the space left in the sector
	if (NumToWrite <= secremain)
		secremain = NumToWrite; //Not larger than this sector range
	while (1)
	{
		Flash_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, Flash_BUF, STM_SECTOR_SIZE / 2); //Read out the whole content of the sector
		for (i = 0; i < secremain; i++)															 //Check the data
		{
			if (Flash_BUF[secoff + i] != 0XFFFF)
				break; //Erase needed
		}
		if (i < secremain) //Erase needed
		{
			Flash_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE); //Erase this sector
			for (i = 0; i < secremain; i++)								  //Copy
			{
				Flash_BUF[i + secoff] = pBuffer[i];
			}
			Flash_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, Flash_BUF, STM_SECTOR_SIZE / 2); //Write the whole sector
		}
		else
			Flash_Write_NoCheck(WriteAddr, pBuffer, secremain); //Already erased, write directly into the remaining area of the sector.
		if (NumToWrite == secremain)
			break; //Write finished
		else	   //Write not finished
		{
			secpos++;					//Sector address +1
			secoff = 0;					//Offset position set to 0
			pBuffer += secremain;		//Pointer offset
			WriteAddr += secremain * 2; //Write address offset (16-bit data address, needs *2)
			NumToWrite -= secremain;	//Half word (16-bit) count decremented
			if (NumToWrite > (STM_SECTOR_SIZE / 2))
				secremain = STM_SECTOR_SIZE / 2; //The next sector still cannot hold it all
			else
				secremain = NumToWrite; //The next sector can hold the rest
		}
	};
	Flash_Lock(); //Lock
}
#endif

//Read data of a given length starting at the specified address
//ReadAddr: start address
//pBuffer: data pointer
//NumToWrite: number of half words (16-bit)
void Flash_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
	uint16_t i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = Flash_ReadHalfWord(ReadAddr);   //Read 2 bytes.
		ReadAddr += 2;								 //Offset by 2 bytes.
	}
}


//Erase a page
//paddr: page address
//Return value: execution status
uint8_t Flash_ErasePage(uint32_t paddr)
{
	uint8_t res = 0;
	res = Flash_WaitDone(0X5FFF); //Wait for the end of the last operation, >20ms
	if (res == 0)
	{
		FLASH->CR |= 1 << 1;			 //Page erase
		FLASH->AR = paddr;				 //Set the page address
		FLASH->CR |= 1 << 6;			 //Start the erase
		res = Flash_WaitDone(0X5FFF); //Wait for the end of the operation, >20ms
		if (res != 1)					 //Not busy
		{
			FLASH->CR &= ~(1 << 1); //Clear the page erase flag.
		}
	}
	return res;
}

