obj-y += flat32.o
obj-y += rom_startup.o

cflags-y += -O0
cflags-y += -c -fmessage-length=0 -mtune=pentium -march=pentium
cflags-y += -static -nodefaultlibs -Wno-ignored-qualifiers
cflags-y += -mno-ms-bitfields

asflags-y += -imacros fst_constants.h -m32 -march=pentium
asflags-y += -x assembler-with-cpp -m32 -march=pentium
