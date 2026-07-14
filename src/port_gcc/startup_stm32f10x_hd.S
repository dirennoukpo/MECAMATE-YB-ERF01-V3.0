/**
  ******************************************************************************
  * @file      startup_stm32f10x_hd.S
  * @brief     GNU as (arm-none-eabi) startup for STM32F103RC / High Density.
  *
  *            1:1 transposition of tmp/V3.5.1/CMSIS/startup_stm32f10x_hd.s
  *            (armasm syntax, which GNU as cannot assemble).
  *
  *            /!\ CRITICAL POINT OF THE PORT /!\
  *            Yahboom hand-patched the vector table to wire it directly onto the
  *            FreeRTOS handlers. FreeRTOSConfig.h contains NO
  *            #define vPortSVCHandler / xPortPendSVHandler / xPortSysTickHandler:
  *            the vector table, and it alone, does the routing. Slots 11 (SVCall),
  *            14 (PendSV) and 15 (SysTick) therefore point at the FreeRTOS
  *            symbols, not at the weak SVC_Handler / PendSV_Handler /
  *            SysTick_Handler.
  *            Using a stock GNU startup here would boot the firmware and then
  *            freeze it at the first context switch, with no error message.
  *
  *            Base: startup_stm32f103xe.s (STMicroelectronics, BSD-3-Clause),
  *            whose init order (SystemInit, then .data/.bss copy) matches the
  *            Keil Reset_Handler (BLX SystemInit, then __main).
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* Symbols provided by the linker script (ldscript.ld) */
.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:

/* Clock setup (PLL from HSE 8 MHz -> SYSCLK 72 MHz) */
  bl  SystemInit

/* Copy .data from FLASH to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero-fill .bss (which includes FreeRTOS heap_2's ucHeap[17408]) */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Static constructors (including the unbuffered stdio init in syscalls.c) */
  bl __libc_init_array
/* Application entry point */
  bl main
  bx lr
.size Reset_Handler, .-Reset_Handler

/**
 * @brief Default handler: infinite loop, state preserved for the debugger.
 */
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
* STM32F10x High Density vector table.
* Verbatim transcription of startup_stm32f10x_hd.s (__Vectors ... __Vectors_End).
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack                     /* Top of Stack           */
  .word Reset_Handler               /* Reset Handler          */
  .word NMI_Handler                 /* NMI Handler            */
  .word HardFault_Handler           /* Hard Fault Handler     */
  .word MemManage_Handler           /* MPU Fault Handler      */
  .word BusFault_Handler            /* Bus Fault Handler      */
  .word UsageFault_Handler          /* Usage Fault Handler    */
  .word 0                           /* Reserved               */
  .word 0                           /* Reserved               */
  .word 0                           /* Reserved               */
  .word 0                           /* Reserved               */
  .word vPortSVCHandler             /* SVCall Handler   <- FreeRTOS (Yahboom patch) */
  .word DebugMon_Handler            /* Debug Monitor Handler  */
  .word 0                           /* Reserved               */
  .word xPortPendSVHandler          /* PendSV Handler   <- FreeRTOS (Yahboom patch) */
  .word xPortSysTickHandler         /* SysTick Handler  <- FreeRTOS (Yahboom patch) */

  /* External Interrupts */
  .word WWDG_IRQHandler             /* Window Watchdog        */
  .word PVD_IRQHandler              /* PVD through EXTI Line detect */
  .word TAMPER_IRQHandler           /* Tamper                 */
  .word RTC_IRQHandler              /* RTC                    */
  .word FLASH_IRQHandler            /* Flash                  */
  .word RCC_IRQHandler              /* RCC                    */
  .word EXTI0_IRQHandler            /* EXTI Line 0            */
  .word EXTI1_IRQHandler            /* EXTI Line 1            */
  .word EXTI2_IRQHandler            /* EXTI Line 2            */
  .word EXTI3_IRQHandler            /* EXTI Line 3            */
  .word EXTI4_IRQHandler            /* EXTI Line 4            */
  .word DMA1_Channel1_IRQHandler    /* DMA1 Channel 1         */
  .word DMA1_Channel2_IRQHandler    /* DMA1 Channel 2         */
  .word DMA1_Channel3_IRQHandler    /* DMA1 Channel 3         */
  .word DMA1_Channel4_IRQHandler    /* DMA1 Channel 4         */
  .word DMA1_Channel5_IRQHandler    /* DMA1 Channel 5         */
  .word DMA1_Channel6_IRQHandler    /* DMA1 Channel 6         */
  .word DMA1_Channel7_IRQHandler    /* DMA1 Channel 7         */
  .word ADC1_2_IRQHandler           /* ADC1 & ADC2            */
  .word USB_HP_CAN1_TX_IRQHandler   /* USB High Priority or CAN1 TX */
  .word USB_LP_CAN1_RX0_IRQHandler  /* USB Low  Priority or CAN1 RX0 */
  .word CAN1_RX1_IRQHandler         /* CAN1 RX1               */
  .word CAN1_SCE_IRQHandler         /* CAN1 SCE               */
  .word EXTI9_5_IRQHandler          /* EXTI Line 9..5         */
  .word TIM1_BRK_IRQHandler         /* TIM1 Break             */
  .word TIM1_UP_IRQHandler          /* TIM1 Update            */
  .word TIM1_TRG_COM_IRQHandler     /* TIM1 Trigger and Commutation */
  .word TIM1_CC_IRQHandler          /* TIM1 Capture Compare   */
  .word TIM2_IRQHandler             /* TIM2                   */
  .word TIM3_IRQHandler             /* TIM3                   */
  .word TIM4_IRQHandler             /* TIM4                   */
  .word I2C1_EV_IRQHandler          /* I2C1 Event             */
  .word I2C1_ER_IRQHandler          /* I2C1 Error             */
  .word I2C2_EV_IRQHandler          /* I2C2 Event             */
  .word I2C2_ER_IRQHandler          /* I2C2 Error             */
  .word SPI1_IRQHandler             /* SPI1                   */
  .word SPI2_IRQHandler             /* SPI2                   */
  .word USART1_IRQHandler           /* USART1                 */
  .word USART2_IRQHandler           /* USART2                 */
  .word USART3_IRQHandler           /* USART3                 */
  .word EXTI15_10_IRQHandler        /* EXTI Line 15..10       */
  .word RTCAlarm_IRQHandler         /* RTC Alarm through EXTI Line */
  .word USBWakeUp_IRQHandler        /* USB Wakeup from suspend */
  .word TIM8_BRK_IRQHandler         /* TIM8 Break             */
  .word TIM8_UP_IRQHandler          /* TIM8 Update            */
  .word TIM8_TRG_COM_IRQHandler     /* TIM8 Trigger and Commutation */
  .word TIM8_CC_IRQHandler          /* TIM8 Capture Compare   */
  .word ADC3_IRQHandler             /* ADC3                   */
  .word FSMC_IRQHandler             /* FSMC                   */
  .word SDIO_IRQHandler             /* SDIO                   */
  .word TIM5_IRQHandler             /* TIM5                   */
  .word SPI3_IRQHandler             /* SPI3                   */
  .word UART4_IRQHandler            /* UART4                  */
  .word UART5_IRQHandler            /* UART5                  */
  .word TIM6_IRQHandler             /* TIM6                   */
  .word TIM7_IRQHandler             /* TIM7                   */
  .word DMA2_Channel1_IRQHandler    /* DMA2 Channel1          */
  .word DMA2_Channel2_IRQHandler    /* DMA2 Channel2          */
  .word DMA2_Channel3_IRQHandler    /* DMA2 Channel3          */
  .word DMA2_Channel4_5_IRQHandler  /* DMA2 Channel4 & Channel5 */

/******************************************************************************
* Weak aliases: every handler points at Default_Handler until the application
* provides a strong definition of its own.
*
* NOTE: SVC_Handler / PendSV_Handler / SysTick_Handler are deliberately NOT
* declared here. They appear nowhere in the vector table (see above); declaring
* them would only create dead symbols, exactly as in the original Keil file.
* The routing goes through the FreeRTOS symbols.
******************************************************************************/

  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak PVD_IRQHandler
  .thumb_set PVD_IRQHandler,Default_Handler

  .weak TAMPER_IRQHandler
  .thumb_set TAMPER_IRQHandler,Default_Handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

  .weak ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler,Default_Handler

  .weak USB_HP_CAN1_TX_IRQHandler
  .thumb_set USB_HP_CAN1_TX_IRQHandler,Default_Handler

  .weak USB_LP_CAN1_RX0_IRQHandler
  .thumb_set USB_LP_CAN1_RX0_IRQHandler,Default_Handler

  .weak CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler,Default_Handler

  .weak CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler,Default_Handler

  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Default_Handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,Default_Handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,Default_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Default_Handler

  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Default_Handler

  .weak RTCAlarm_IRQHandler
  .thumb_set RTCAlarm_IRQHandler,Default_Handler

  .weak USBWakeUp_IRQHandler
  .thumb_set USBWakeUp_IRQHandler,Default_Handler

  .weak TIM8_BRK_IRQHandler
  .thumb_set TIM8_BRK_IRQHandler,Default_Handler

  .weak TIM8_UP_IRQHandler
  .thumb_set TIM8_UP_IRQHandler,Default_Handler

  .weak TIM8_TRG_COM_IRQHandler
  .thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

  .weak TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler,Default_Handler

  .weak ADC3_IRQHandler
  .thumb_set ADC3_IRQHandler,Default_Handler

  .weak FSMC_IRQHandler
  .thumb_set FSMC_IRQHandler,Default_Handler

  .weak SDIO_IRQHandler
  .thumb_set SDIO_IRQHandler,Default_Handler

  .weak TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler,Default_Handler

  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler,Default_Handler

  .weak TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler,Default_Handler

  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak DMA2_Channel1_IRQHandler
  .thumb_set DMA2_Channel1_IRQHandler,Default_Handler

  .weak DMA2_Channel2_IRQHandler
  .thumb_set DMA2_Channel2_IRQHandler,Default_Handler

  .weak DMA2_Channel3_IRQHandler
  .thumb_set DMA2_Channel3_IRQHandler,Default_Handler

  .weak DMA2_Channel4_5_IRQHandler
  .thumb_set DMA2_Channel4_5_IRQHandler,Default_Handler
