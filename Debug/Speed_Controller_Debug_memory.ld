/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2021
 * Generated linker script file for MKV31F256xxx12
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v11.0.1 [Build 2563] [2019-09-01] on Feb 1, 2021 4:23:36 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x40000 /* 256K bytes (alias Flash) */  
  SRAM_UPPER (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32K bytes (alias RAM) */  
  SRAM_LOWER (rwx) : ORIGIN = 0x1fffc000, LENGTH = 0x4000 /* 16K bytes (alias RAM2) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x40000 ; /* 256K bytes */  
  __top_Flash = 0x0 + 0x40000 ; /* 256K bytes */  
  __base_SRAM_UPPER = 0x20000000  ; /* SRAM_UPPER */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM_UPPER = 0x20000000 + 0x8000 ; /* 32K bytes */  
  __top_RAM = 0x20000000 + 0x8000 ; /* 32K bytes */  
  __base_SRAM_LOWER = 0x1fffc000  ; /* SRAM_LOWER */  
  __base_RAM2 = 0x1fffc000 ; /* RAM2 */  
  __top_SRAM_LOWER = 0x1fffc000 + 0x4000 ; /* 16K bytes */  
  __top_RAM2 = 0x1fffc000 + 0x4000 ; /* 16K bytes */  
