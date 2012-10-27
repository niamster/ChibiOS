# List of the ChibiOS/RT MIPS32r2 port files.
PORTSRC = ${CHIBIOS}/os/ports/GCC/MIPS/chcore.c

PORTASM = ${CHIBIOS}/os/ports/GCC/MIPS/MIPS32r2/context.s \
          ${CHIBIOS}/os/ports/GCC/MIPS/MIPS32r2/vectors.s

PORTINC = ${CHIBIOS}/os/ports/GCC/MIPS \
          ${CHIBIOS}/os/ports/GCC/MIPS/MIPS32r2
