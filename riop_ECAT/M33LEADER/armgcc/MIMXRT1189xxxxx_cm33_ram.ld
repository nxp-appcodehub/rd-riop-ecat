/*
** ###################################################################
**     Processors:          MIMXRT1189CVM8B_cm33
**                          MIMXRT1189CVM8C_cm33
**                          MIMXRT1189XVM8B_cm33
**                          MIMXRT1189XVM8C_cm33
**
**     Compiler:            GNU C Compiler
**     Reference manual:    IMXRT1180RM, Rev 5, 01/2024
**     Version:             rev. 2.0, 2024-01-18
**     Build:               b250310
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2025 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
** ###################################################################
*/

/* Entry Point */
ENTRY(Reset_Handler)

__ram_vector_table__ = 1;

/* Board memory map */
m_code_tcm_start         = 0x0FFE0000;
m_code_tcm_size          = 0x00020000;
m_system_tcm_start       = 0x20000000;
m_system_tcm_size        = 0x00020000;
m_ocram1_start           = 0x20484000;  /* OCRAM1 first 16K access is blocked by TRDC */
m_ocram1_size            = 0x0007C000;
m_ocram2_start           = 0x20500000;
m_ocram2_size            = 0x00040000;

/* CM33 use last 2M bytes space of sdram and hyperram for dual image framework */
m_sdram_start            = DEFINED(__dual_image__) ? 0x81E00000 : 0x80000000;
m_sdram_size             = DEFINED(__dual_image__) ? 0x00200000 : 0x02000000;
m_hyperram_start         = DEFINED(__dual_image__) ? 0x04600000 : 0x04000000;
m_hyperram_size          = DEFINED(__dual_image__) ? 0x00200000 : 0x00800000;
m_flash_start            = 0x28000000;
m_flash_size             = 0x00800000;

m_core1_image_maximum_size = 0x00040000;

/* General definition */
app_image_offset         = DEFINED(__dual_image__) ? 0x0010B000 : 0x0000B000;
vector_table_size        = 0x00000400;
m_qacode_start           = m_code_tcm_start + (DEFINED(__ram_vector_table__) ? vector_table_size : 0);
m_qacode_end             = m_code_tcm_start + m_code_tcm_size - 1;
m_qadata_start           = m_system_tcm_start;
m_qadata_end             = m_system_tcm_start + m_system_tcm_size - 1;
m_ram_vector_table_start = m_code_tcm_start;
m_stack_size             = DEFINED(__stack_size__) ? __stack_size__ : 0x2000;
m_heap_size              = DEFINED(__heap_size__)  ? __heap_size__  : 0x0400;

/* Target specific definition, code & data allocation */
m_code_size              = DEFINED(__multicore__) ? 0 : m_ocram2_size;
m_data_size              = m_system_tcm_size;
m_ncache_size            = DEFINED(__multicore__) ? 0x20000 : 0x20000;   /* m_ncache_size must be 2^N */
m_shmem_size             = DEFINED(__multicore__) ? 0x2000 : 0;  /* m_shmem_size must be 2^N */

m_ram_vector_table_size  = DEFINED(__ram_vector_table__) ? vector_table_size : 0;

m_interrupts_start       = m_code_tcm_start;
m_interrupts_end         = m_interrupts_start + vector_table_size - 1;

m_text_start             = DEFINED(__multicore__) ? 0 : m_ocram2_start;
m_text_end               = m_text_start + m_code_size - 1;

m_data_start             = m_system_tcm_start;
m_data_end               = m_data_start + m_data_size - 1;

/* OCRAM1 Start and Ends */
m_heap_start             = m_ocram1_start;
m_heap_end               = m_heap_start + m_heap_size - 1;

m_ncache_start           = m_heap_end + 1;
m_ncache_end             = m_ncache_start + m_ncache_size - 1;

m_cache_start            = m_ncache_end + 1;
m_cache_end              = m_ocram1_start + m_ocram1_size - m_shmem_size - 1;

m_rpmsg_sh_mem_start     = m_ocram1_start + m_ocram1_size - m_shmem_size;
m_rpmsg_sh_mem_end       = m_rpmsg_sh_mem_start + m_shmem_size - 1;

/* OCRAM2 Start and Ends */
m_ocram2_cm7_start       = DEFINED(__multicore__) ? m_ocram2_start : 0;
m_ocram2_cm7_end         = DEFINED(__multicore__) ? m_ocram2_start + m_ocram2_size - 1 : 0; 


/* Exported symbol definition */
__VECTOR_TABLE                = m_interrupts_start;
__VECTOR_RAM                  = m_ram_vector_table_start;
__RAM_VECTOR_TABLE_SIZE_BYTES = m_ram_vector_table_size;

__NCACHE_REGION_START = m_ncache_start;
__NCACHE_REGION_SIZE  = m_ncache_size;
PROVIDE(__RPMSG_SH_MEM_START  = DEFINED(__use_shmem__)? m_rpmsg_sh_mem_start : 0);
PROVIDE(__RPMSG_SH_MEM_SIZE   = DEFINED(__use_shmem__)? m_rpmsg_sh_mem_end - m_rpmsg_sh_mem_start + 1 : 0);

/* Specify the memory areas */
MEMORY
{
  m_interrupts_ram (RX)  : ORIGIN = m_ram_vector_table_start, LENGTH = __RAM_VECTOR_TABLE_SIZE_BYTES
  m_interrupts     (RX)  : ORIGIN = m_interrupts_start,       LENGTH = m_interrupts_end - m_interrupts_start + 1
  m_text           (RX)  : ORIGIN = m_text_start,             LENGTH = DEFINED(__multicore__) ? 0 : m_text_end - m_text_start + 1
  m_data           (RW)  : ORIGIN = m_data_start,             LENGTH = m_data_end - m_data_start + 1
  m_ncache         (RW)  : ORIGIN = m_ncache_start,           LENGTH = m_ncache_end - m_ncache_start + 1
  m_heap           (RW)  : ORIGIN = m_heap_start,             LENGTH = m_heap_end - m_heap_start + 1
  m_cache          (RW)  : ORIGIN = m_cache_start,            LENGTH = m_cache_end - m_cache_start + 1
  m_qacode         (RX)  : ORIGIN = m_qacode_start,           LENGTH = m_qacode_end - m_qacode_start + 1
  m_qadata         (RW)  : ORIGIN = m_qadata_start,           LENGTH = m_qadata_end - m_qadata_start + 1
  m_rpmsg          (RW)  : ORIGIN = m_rpmsg_sh_mem_start,     LENGTH = DEFINED(__multicore__) ? m_rpmsg_sh_mem_end - m_rpmsg_sh_mem_start + 1 : 0
  m_ocram2_cm7     (RW)  : ORIGIN = m_ocram2_cm7_start,       LENGTH = DEFINED(__multicore__) ? (m_ocram2_cm7_end - m_ocram2_cm7_start + 1) : 0
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first */
  .interrupts :
  {
    . = ALIGN(4);
    __Vectors = .;
    KEEP(*(.isr_vector))     /* Vector table and startup code */
    . = ALIGN(4);
  } > m_interrupts

  .interrupts_ram :
  {
    . = ALIGN(4);
    . += __RAM_VECTOR_TABLE_SIZE_BYTES;
    . = ALIGN(4);
  } > m_interrupts_ram

  /* Place some goal code into external ocm memory section m_ncache */
  .goal_text :
  {
    . = ALIGN(4);
    */goal_media/*.* (.text .text*)
    */protos/*.* (.text .text*)
    */ext/*.* (.text .text*)
    . = ALIGN(4);
  } > m_text

  .goal_data :
  {
    . = ALIGN(4);
    */goal/*.* (.rodata .rodata* )
    */goal_media/*.* (.rodata .rodata*)
    */protos/*.* (.rodata .rodata*)
    */ext/*.* (.rodata .rodata*)
    . = ALIGN(4);
  } > m_data

  /* The program code */
  .text (READONLY) :
  {
    . = ALIGN(4);

    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */

    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
  } > m_qacode

  .ARM.extab (READONLY) :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_qacode

  .ARM (READONLY) :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_qacode

  .ctors (READONLY) :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.* to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.*; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.*
       is in.  */
    KEEP (*crtbegin.*(.ctors))
    KEEP (*crtbegin?.*(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.* file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.* *crtend.*) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_qacode

  .dtors (READONLY) :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.*(.dtors))
    KEEP (*crtbegin?.*(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.* *crtend.*) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_qacode

  .preinit_array (READONLY) :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_qacode

  .init_array (READONLY) :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_qacode

  .fini_array (READONLY) :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_qacode

  __etext = .;    /* define a global symbol at end of code */

  __data_flash_start = .; /* Symbol is used by startup for data initialization */
  .data : AT(__data_flash_start)
  {
    . = ALIGN(4);
    __data_start__ = .;      /* create a global symbol at data start */
    *(m_usb_dma_init_data)
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    *(DataQuickAccess)       /* quick access data section */
    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data

  __ram_function_flash_start = __data_flash_start + (__data_end__ - __data_start__); /* Symbol is used by startup for TCM code initialization */
  .ram_function : AT(__ram_function_flash_start)
  {
    . = ALIGN(32);
    __ram_function_start__ = .;
    *(CodeQuickAccess)
    . = ALIGN(128);
    __ram_function_end__ = .;
  } > m_qacode

  __noncache_data_flash_start = __ram_function_flash_start + (__ram_function_end__ - __ram_function_start__);
  .ncache.init : AT(__noncache_data_flash_start)
  {
    . = ALIGN(4);
    __noncache_data_start__ = .;      /* create a global symbol at ncache data start */
    *(NonCacheable.init)
    . = ALIGN(4);
    __noncache_data_end__ = .;   /* create a global symbol at ncache data end */
  } > m_ncache

  .ncache :
  {
    . = ALIGN(4);
    __noncache_bss_start__ = .;  /* define a global symbol at ncache bss start */
    *(NonCacheable)
    . = ALIGN(4);
    __noncache_bss_end__ = .;    /* define a global symbol at ncache bss end */
  } > m_ncache

  __qadata_flash_start = __noncache_data_flash_start + (__noncache_data_end__ - __noncache_data_start__);
  .qadata : AT(__qadata_flash_start)
  {
    . = ALIGN(4);
    __qadata_start__ = .;
    . = ALIGN(4);
    __qadata_end__ = .;
  } > m_qadata

  text_data_end = __qadata_flash_start + (__qadata_end__ - __qadata_start__);
  text_end = ORIGIN(m_text) + LENGTH(m_text);
  ASSERT(text_data_end < text_end, "region m_text overflowed with text and data")

  /* map freertos (which includes GOAL heap) to m_ncache region, since it exceeds .bss */
  .freertos_heap :
  {
    . = ALIGN(4);
    _freertos_heap_start__ = .;
    *(.bss.memHeap)
    *(.bss.ucHeap)
    . = ALIGN(4);
    _freertos_heap_end__ = .;
  } > m_cache

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __bss_start__ = .;
    *(m_usb_dma_noninit_data)
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > m_ncache

  .rpmsg :
  {
    KEEP(*(.noinit.$rpmsg_sh_mem))
  } > m_rpmsg

  .heap :
  {
    . = ALIGN(8);
    __HeapBase = .;
    end = .;
    /* totol system heap (this does not include the freeRTOS HEAP) */
    . += m_heap_size;
    __HeapLimit = .;  /* Add for _sbrk */
  } > m_heap

  .stack :
  {
    . = ALIGN(8);
    __StackStart = .;
    . += m_stack_size;
  } > m_data

  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(m_data) + LENGTH(m_data);
  __StackLimit = __StackTop - m_stack_size;
  PROVIDE(__stack = __StackTop);
  PROVIDE (__stack_size = m_stack_size);
  ASSERT(__StackLimit >= __StackStart, "region m_data overflowed with stack and heap")

  .ARM.attributes 0 : { *(.ARM.attributes) }
}