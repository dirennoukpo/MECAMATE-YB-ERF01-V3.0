/**
 ******************************************************************************
 * @file    syscalls.c
 * @brief   Newlib retarget for the GCC port of the YB-ERF01 firmware.
 *
 * Keil built against MicroLIB (<useUlib>1</useUlib>), where printf() calls
 * fputc() directly. Source/BSP/bsp_usart.c:462 defines that fputc(), and
 * bsp_usart.c:474 the matching fgetc().
 *
 * Newlib NEVER calls fputc/fgetc: it goes through the _write/_read syscalls.
 * Without this file the firmware compiles and links without a single warning,
 * yet EVERY printf() is silently dropped.
 *
 * bsp_usart.c is left untouched: its fputc/fgetc simply become dead code and
 * are dropped at link time by --gc-sections. The register sequence is
 * reproduced here identically.
 ******************************************************************************
 */

/*
 * NOTE: neither <unistd.h> nor <signal.h> here. They declare prototypes taking
 * pid_t, but the project builds with -D_PID_T_DECLARED (see platformio.ini):
 * newlib's POSIX pid_t is suppressed in favour of the project's own pid_t
 * (the PID controller struct in Source/APP/app_pid.h). The syscalls are
 * therefore declared by hand.
 */
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/stat.h>

#include "stm32f10x.h"
#include "bsp_usart.h"

#include "FreeRTOS.h"
#include "task.h"

/* Provided by ldscript.ld */
extern char end; /* first free byte past .bss */
extern char _estack;
extern char _Min_Stack_Size;

/* ------------------------------------------------------------------------- */
/* stdout / stderr -> DEBUG_USARTx (USART1 @115200, see bsp_usart.h)          */
/* ------------------------------------------------------------------------- */

/*
 * Exact reproduction of fputc() (bsp_usart.c:462-471): write DR, THEN wait for
 * TXE. The order is unusual (USART1_Send_U8 does the opposite), but it is what
 * the reference binary does, so we do not "fix" it.
 *
 * Reminder of a PRE-EXISTING defect, not introduced here: USART1 has its TX on
 * DMA (DMAT is armed once and for all in USART1_Init) and printf writes to the
 * same DR from the CPU, with no mutual exclusion. A printf landing during a DMA
 * transfer interleaves the two streams. Keil already behaved this way.
 */
int _write(int fd, char *ptr, int len)
{
	int i;

	(void)fd;

	for (i = 0; i < len; i++)
	{
		USART_SendData(DEBUG_USARTx, (uint8_t)ptr[i]);

		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
			;
	}

	return len;
}

/* Exact reproduction of fgetc() (bsp_usart.c:474-480): blocking wait on RXNE. */
int _read(int fd, char *ptr, int len)
{
	(void)fd;

	if (len <= 0)
		return 0;

	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET)
		;

	ptr[0] = (char)USART_ReceiveData(DEBUG_USARTx);

	return 1;
}

/*
 * MicroLIB does no buffering: printf() emits character by character. Newlib does
 * buffer, and with _isatty()==0 it would switch to "fully buffered" mode: output
 * would only appear once 1024 bytes had accumulated. We therefore force stdout
 * and stderr unbuffered to recover the original behaviour -- and, incidentally,
 * to avoid the 1 KB malloc of the stdio buffer.
 *
 * This constructor is called by __libc_init_array(), from Reset_Handler, before
 * main() (hence before Bsp_Init: no hardware access here).
 */
__attribute__((constructor)) static void yb_stdio_init(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
}

/* ------------------------------------------------------------------------- */
/* Newlib heap                                                                */
/* ------------------------------------------------------------------------- */

/*
 * The newlib heap is NOT the FreeRTOS heap: the latter is ucHeap[17408], a
 * static array in heap_2.c placed in .bss (configTOTAL_HEAP_SIZE).
 *
 * The newlib heap only serves the C library, and it IS required: floating-point
 * conversion (_dtoa_r/_Balloc, pulled in by the sprintf("%.1f") calls in
 * app_oled.c and the printf("%.3f") in app_flash.c) calls malloc(). An _sbrk()
 * returning -1 would make those conversions fail silently.
 *
 * We deliberately do NOT use libnosys's _sbrk: it compares the top of the heap
 * against the current stack pointer (SP). Under FreeRTOS a task runs on a stack
 * allocated inside ucHeap, i.e. at an address BELOW `end`, so the test would
 * wrongly fail and malloc() would return NULL as soon as it was called from a
 * task. We therefore bound the heap explicitly with the main stack (MSP).
 */
void *_sbrk(ptrdiff_t incr)
{
	static char *heap_end = NULL;
	char *prev_heap_end;
	char *heap_limit = &_estack - (ptrdiff_t)&_Min_Stack_Size;

	if (heap_end == NULL)
		heap_end = &end;

	prev_heap_end = heap_end;

	if (heap_end + incr > heap_limit)
	{
		errno = ENOMEM;
		return (void *)-1;
	}

	heap_end += incr;

	return (void *)prev_heap_end;
}

/*
 * The newlib heap is shared by every task (the periodic sprintf("%.1f") of the
 * OLED task, the printf("%.3f") of Flash_Init...). malloc() is not reentrant:
 * without a lock, two concurrent float conversions would corrupt the heap.
 * MicroLIB did not have this problem (no malloc for %f).
 *
 * The lock masks interrupts through BASEPRI, and explicitly NOT through the
 * scheduler API. Three reasons, all verified:
 *
 *  - vTaskSuspendAll()/xTaskResumeAll() are FreeRTOS calls that are forbidden
 *    from an ISR whose priority is above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 *    Yet the CAN ISR (bsp_can.c:129, NVIC priority 0x90 < threshold 0xBF) runs
 *    all the way down to Upper_CAN_Execute_Command -> Flash_Init ->
 *    printf("%.3f") -> malloc. Calling them there would corrupt the scheduler.
 *
 *  - PRIMASK (__disable_irq) would be worse: TIM7 generates the servo PWM in
 *    software every 10 us (bsp_timer.c:42-43, prescaler 71 / period 9). Masking
 *    every interrupt for the duration of a malloc would break that PWM.
 *
 *  - BASEPRI at configMAX_SYSCALL_INTERRUPT_PRIORITY masks ONLY PendSV and
 *    SysTick (priority 0xFF): no application ISR is delayed (they all sit above
 *    the threshold: TIM7 0x50, USART1 0x70, CAN 0x90), yet no context switch can
 *    occur. That is exactly what is needed.
 *
 * portSET/CLEAR_INTERRUPT_MASK_FROM_ISR save and restore the previous BASEPRI:
 * usable from a task, from an ISR, and before the scheduler is started.
 */
static uint32_t s_malloc_basepri;
static uint32_t s_malloc_nesting;

void __malloc_lock(struct _reent *r)
{
	uint32_t saved;

	(void)r;

	saved = portSET_INTERRUPT_MASK_FROM_ISR();

	if (s_malloc_nesting == 0)
		s_malloc_basepri = saved;

	s_malloc_nesting++;
}

void __malloc_unlock(struct _reent *r)
{
	(void)r;

	if (s_malloc_nesting > 0 && --s_malloc_nesting == 0)
		portCLEAR_INTERRUPT_MASK_FROM_ISR(s_malloc_basepri);
}

/* ------------------------------------------------------------------------- */
/* POSIX stubs required by newlib at link time                                */
/* ------------------------------------------------------------------------- */

int _close(int fd)
{
	(void)fd;
	return -1;
}

int _fstat(int fd, struct stat *st)
{
	(void)fd;
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int fd)
{
	(void)fd;
	return 1;
}

int _lseek(int fd, int offset, int whence)
{
	(void)fd;
	(void)offset;
	(void)whence;
	return -1;
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig;
	errno = EINVAL;
	return -1;
}

void _exit(int status)
{
	(void)status;
	while (1)
		;
}
