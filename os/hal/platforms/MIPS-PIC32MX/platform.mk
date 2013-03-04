# List of all the MIPS-PIC32MX platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/hal_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/eic_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/serial_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/pal_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/spi_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/usb_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/ext_lld.c \
              ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/dma_lld.c

PLATFORMASM = ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX/mcu/pic32mxxx.s

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/platforms/MIPS-PIC32MX
