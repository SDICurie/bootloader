/* board feature partition*/
. = ORIGIN(BOARD_FEATURES);
. = ALIGN(4);

.board_features (NOLOAD):
{
__board_features_start = .;
/* The first 32 bits are used later on by ARC to get the boot address */
. = . + 4;
/* Here we will pass the hardware features  */
KEEP(*(.board_features))
. = ALIGN(4);
__board_features_end = .;
} > BOARD_FEATURES
__board_features_num_words = ( __board_features_end - __board_features_start ) >> 2;

/* usb shared if partition*/
. = ORIGIN(USB_SHARED);
. = ALIGN(4);

.usb_shared_if (NOLOAD):
{
__usb_shared_start = .;
KEEP(*(.usb_shared_if))
. = ALIGN(4);
__usb_shared_end = .;
} > USB_SHARED
__usb_shared_num_words = ( __usb_shared_end - __usb_shared_start ) >> 2;

/* security shared if partition*/
. = ORIGIN(SECURITY_SHARED);
. = ALIGN(4);

.security_shared_if : AT ( _init_shared_data_start )
{
__security_shared_data_start = .;
__security_shared_data_lma = LOADADDR(.security_shared_if); __security_shared_data_vma = .;
/* Here we will pass the hardware features  */
KEEP(*(.security_shared_if))
KEEP(*(.data.uecc*))
KEEP(*(.data.sha256*))
. = ALIGN(4);
} > SECURITY_SHARED

__security_shared_data_size = SIZEOF( .security_shared_if );

.bss_security_shared_if (NOLOAD):
{
__bss_security_shared_start = .;
*(.bss.uecc*)
*(.bss.sha256*)
. = ALIGN(4);
__bss_security_shared_end = .;
} > SECURITY_SHARED

__bss_security_shared_num_words = (__bss_security_shared_end - __bss_security_shared_start) >> 2;

_init_shared_data_end = _init_shared_data_start + __security_shared_data_size;
