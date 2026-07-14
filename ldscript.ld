/*
******************************************************************************
** GCC linker script — STM32F103RC (YB-ERF01)
**
** Reproduces the memory declared in rosmaster.uvprojx (line 21):
**   IROM(0x08000000, 0x40000)  -> FLASH 256K
**   IRAM(0x20000000, 0x0C000)  -> RAM    48K
** The Keil project has NO scatter file (<ScatterFile> empty, umfTarg=1):
** armlink generated the layout from those two ranges. There was therefore
** nothing to convert; this script follows the standard STM32CubeIDE template.
**
** _Min_Heap_Size = 0x200 matches the Heap_Size EQU of the Keil startup
** (startup_stm32f10x_hd.s:45). _Min_Stack_Size is raised above Keil's 0x400,
** see the note below.
**
** Beware of the three distinct "heaps":
**   - ucHeap[17408] (FreeRTOS heap_2): a static array, it lives in .bss.
**   - the newlib heap: starts at `end`, grows upwards, bounded by _sbrk() at
**     (_estack - _Min_Stack_Size). Used only by the C library (%f conversion).
**   - the main stack (MSP): starts at _estack and grows down. Used before the
**     scheduler starts and by every ISR. Task stacks, by contrast, live inside
**     ucHeap.
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* 0x2000C000 */

_Min_Heap_Size = 0x200;  /* cf. Heap_Size EQU 0x00000200 (Keil startup) */

/*
 * Main stack (MSP). Keil reserved 0x400 (Stack_Size EQU, startup line 35).
 * The port raises it to 0x800: ISRs run on MSP, and the CAN ISR (bsp_can.c:129)
 * runs all the way down to Upper_CAN_Execute_Command -> Flash_Init ->
 * printf("%.3f"). Newlib's float formatting costs ~600 bytes more than
 * MicroLIB's: that path measures ~904 bytes, leaving only 120 bytes of headroom
 * in 1 KB. Too tight.
 * Cost: 1 KB of RAM out of the ~23 KB still free. Pure hardening, no behavioural
 * change.
 *
 * _Min_Stack_Size serves two purposes: the link-time space check, and the
 * newlib heap ceiling computed by _sbrk() (src/port_gcc/syscalls.c).
 */
_Min_Stack_Size = 0x800;

/* Memories definition */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 48K
  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 256K
}

/* Sections */
SECTIONS
{
  /* Vector table (src/port_gcc/startup_stm32f10x_hd.S). KEEP is mandatory:
     nothing references it, so --gc-sections would strip it. */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array     :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* __libc_init_array walks this table: it holds the yb_stdio_init()
     constructor from syscalls.c (unbuffered stdout). */
  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)

    . = ALIGN(4);
    _edata = .;

  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* Reservation: guarantees at link time that at least _Min_Heap_Size +
     _Min_Stack_Size of RAM remains free. `end` is the base of the newlib heap
     (see _sbrk). */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
