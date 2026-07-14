#include "bsp_wdg.h"
#include "bsp.h"


// Watchdog initialization, prescaler is 64, reload value is 625, timeout is 1s	
void IWDG_Init(void)
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //Enable write access to the IWDG_PR and IWDG_RLR registers
	
	IWDG_SetPrescaler(4);  //Set the IWDG prescaler value: set the IWDG prescaler to 64
	
	IWDG_SetReload(625);  //Set the IWDG reload value
	
	IWDG_ReloadCounter();  //Reload the IWDG counter with the value of the IWDG reload register
	
	IWDG_Enable();  //Enable the IWDG
}

//Feed the independent watchdog
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}


