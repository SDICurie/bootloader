#include <partition.h>
#include "quark_se/boot_config.h"
#include "machine/soc/intel/quark_se/cos_interface.h"

OUTPUT_ARCH(iamcu:intel)
OUTPUT_FORMAT("elf32-iamcu")
ENTRY(_rom_entry_point)

MEMORY
{
  ROM (r) : ORIGIN = 0xFFFFE400, LENGTH = 7k - 16
  RESETVECTOR (xrw) : ORIGIN = 0xFFFFFFF0, LENGTH = 16
  ESRAM (rw) : ORIGIN = QUARK_RAM_START_ADDR , LENGTH = QUARK_RAM_SIZE*1K
  #include "x86/intel/shared_partitions.lds"
}

SECTIONS
{
  . = ORIGIN(ROM);
   _text_base = .;

  .version_header_section () :
  {
    *(.version_header)
    KEEP(*(.version_header*))
  } > ROM

  .reset_vector :
  {
    . = ALIGN(4);
    KEEP(*start16.o) /* Startup code */
  } > RESETVECTOR

  .text :
  {
   *(.text.entry)
   *(.text)
   *(.text.last)
   *(.text.*)
  } > ROM

  . = ALIGN(4);
  .rdata : ALIGN(4)
  {
   *(.rdata)
   *(.rdata.*)
   _init_shared_data_start = .;
  } > ROM

  #include "x86/intel/shared_interface.lds"

  /*Put data at the bottom of the 80K of RAM*/
  . = ORIGIN(ESRAM);
  . = ALIGN(4);

  /* Data load address is here, but appears in the image after .rdata */
  .data : AT ( _init_shared_data_end)
  {
   __data_lma = LOADADDR(.data); __data_vma = .;
   *(.data)
   *(.data.*)
   . = ALIGN(4);
  } > ESRAM

  __data_size = SIZEOF( .data );

  . = ALIGN(4);
  .bss :
  {
   __bss_start = .;
   *(.bss)
   *(.bss.*)
   *(COMMON)
   __bss_end = .;
  } > ESRAM

  __bss_num_words = (__bss_end - __bss_start) >> 2;


  . = ALIGN(4);
  .eh_frame :
  {
     *(.eh_frame)
  }

  . = ALIGN(4);
  .drectve :
  {
     *(.drectve)
  }

  .comment 0 : { *(.comment) }

    /* Reserve panic section at end of RAM in order to
   * avoid overwriting panic dumps with stack */
  .panic_section ORIGIN(ESRAM) + LENGTH(ESRAM) - PANIC_DUMP_MAX_SIZE:
  {
   __panic_start__ = .;
   . += PANIC_DUMP_MAX_SIZE;
   __panic_end__ = .;
  } > ESRAM

  /* Compute new stack top address */
  __StackTop = ORIGIN(ESRAM) + LENGTH(ESRAM) - SIZEOF(.panic_section);
}

