# List of all the MIPS-PIC32MX platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/hal_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/eic_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/serial_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/pal_lld.c

PLATFORMASM = ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/mcu/pic32mxxx.s

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX
