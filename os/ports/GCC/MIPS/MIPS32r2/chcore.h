/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    MIPS/MIPS32r2/chcore.h
 * @brief   MIPS32r2 architecture port macros and structures.
 *
 * @addtogroup MIPS_CORE
 * @{
 */

#ifndef _CHCORE_H_
#define _CHCORE_H_

#include "mipsparams.h"
#include "mipsasm.h"
#include "mipsarch.h"

/*===========================================================================*/
/* Port constants.                                                           */
/*===========================================================================*/

/* Core variants identifiers.*/

/* Inclusion of the MIPS implementation specific parameters.*/

/* MIPS core check, only MIP32R2 supported right now.*/
#if !( (MIPS_CORE == MIPS_CORE_MIPS32R2) )
#error "unknown or unsupported MIPS core"
#endif

/*===========================================================================*/
/* Port statically derived parameters.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Port macros.                                                              */
/*===========================================================================*/

/*===========================================================================*/
/* Port configurable parameters.                                             */
/*===========================================================================*/

/**
 * @brief   If enabled allows the idle thread to enter a low power mode.
 */
#ifndef MIPS_ENABLE_WFI_IDLE
#define MIPS_ENABLE_WFI_IDLE             TRUE
#endif

/**
 * @brief   If defined shadow registers are used during the exception.
 * @note    This enables vectored interrupts mode[not EIC]
 */
/* #define MIPS_USE_SHADOW_GPR */

/**
 * @brief   If defined vectored interrupt mode is enabled[possibly EIC].
 */
/* #define MIPS_USE_VECTORED_IRQ */

/**
 * @brief   If defined MIPS core timer is handled by the port core code
 */
/* #define MIPS_PORT_HANDLE_CORE_TIMER */

/**
 * @brief   If defined MIPS16 ISA is used to compile C and C++ code
 */
/* #define MIPS_USE_MIPS16_ISA */

#if defined(MIPS_USE_MIPS16_ISA)
#define __nomips16 __attribute__ ((nomips16))
#else
#define __nomips16
#endif

/*===========================================================================*/
/* Port exported info.                                                       */
/*===========================================================================*/

/**
 * @brief   Macro defining a generic MIPS architecture.
 */
#define CH_ARCHITECTURE_MIPS

#if defined(__DOXYGEN__)
/**
 * @brief   Macro defining the specific MIPS architecture.
 * @note    This macro is for documentation only, the real name changes
 *          depending on the selected architecture, the possible names are:
 *          - CH_ARCHITECTURE_MIPS32r2.
 *          .
 */
#define CH_ARCHITECTURE_MIPSxxxx

/**
 * @brief   Name of the implemented architecture.
 * @note    The value is for documentation only, the real value changes
 *          depending on the selected architecture, the possible values are:
 *          - "MIPS32".
 *          .
 */
#define CH_ARCHITECTURE_NAME            "MIPSxx"

/**
 * @brief   Name of the architecture variant (optional).
 * @note    The value is for documentation only, the real value changes
 *          depending on the selected architecture, the possible values are:
 *          - "MIPS32r2"
 *          .
 */
#define CH_CORE_VARIANT_NAME            "MIPSxy"

/**
 * @brief   Port-specific information string.
 * @note    The value is for documentation only, the real value changes
 *          depending on the selected options, the possible values are:
 *          - "Pure MIPS"
 *          .
 */
#define CH_PORT_INFO                    "MIPS"

#elif MIPS_CORE == MIPS_CORE_MIPS32R2
#define CH_ARCHITECTURE_MIPS32R2
#define CH_ARCHITECTURE_NAME            "MIPS32"
#define CH_CORE_VARIANT_NAME            "MIPS32r2"
#endif

#define CH_PORT_INFO                    "Pure MIPS mode"

/**
 * @brief   Name of the compiler supported by this port.
 */
#define CH_COMPILER_NAME                "GCC " __VERSION__

/*===========================================================================*/
/* Port implementation part (common).                                        */
/*===========================================================================*/

/**
 * @brief   32 bits stack and memory alignment enforcement.
 */
typedef uint32_t stkalign_t;

/**
 * @brief   Generic MIPS register.
 */
typedef void *regmips_t;

/**
 * @brief   Interrupt saved context.
 * @details This structure represents the stack frame saved during a
 *          preemption-capable interrupt handler.
 */
struct extctx {
  regmips_t      at;
  regmips_t      v0;
  regmips_t      v1;
  regmips_t      a0;
  regmips_t      a1;
  regmips_t      a2;
  regmips_t      a3;
  regmips_t      t0;
  regmips_t      t1;
  regmips_t      t2;
  regmips_t      t3;
  regmips_t      t4;
  regmips_t      t5;
  regmips_t      t6;
  regmips_t      t7;
  regmips_t      t8;
  regmips_t      t9;
  regmips_t      fp;
  regmips_t      ra;
  regmips_t      lo;
  regmips_t      hi;
  regmips_t      pc;
};

/**
 * @brief   System saved context.
 * @details This structure represents the inner stack frame during a context
 *          switching.
 */
struct intctx {
  regmips_t      s0;
  regmips_t      s1;
  regmips_t      s2;
  regmips_t      s3;
  regmips_t      s4;
  regmips_t      s5;
  regmips_t      s6;
  regmips_t      s7;
  regmips_t      fp;
  regmips_t      ra;
  regmips_t  status;
};

/**
 * @brief   Platform dependent part of the @p Thread structure.
 * @details In this port the structure just holds a pointer to the @p intctx
 *          structure representing the stack pointer at context switch time.
 */
struct context {
  struct intctx *sp;
};

/**
 * @brief   Platform dependent part of the @p chThdCreateI() API.
 * @details This code usually setup the context switching frame represented
 *          by an @p intctx structure.
 */
#define SETUP_CONTEXT(workspace, wsize, pf, arg) {          \
    tp->p_ctx.sp = (struct intctx *)((uint8_t *)workspace + \
        wsize -                                             \
        sizeof(struct intctx));                             \
    tp->p_ctx.sp->s0 = arg;                                 \
    tp->p_ctx.sp->s1 = pf;                                  \
    tp->p_ctx.sp->ra = _port_thread_start;                  \
    tp->p_ctx.sp->status = c0_get_status() | 0x1;           \
  }

/**
 * @brief   Stack size for the system idle thread.
 * @details This size depends on the idle thread implementation, usually
 *          the idle thread should take no more space than those reserved
 *          by @p PORT_INT_REQUIRED_STACK.
 * @note    In this port it is set to 4 because the idle thread does have
 *          a stack frame when compiling without optimizations.
 */
#ifndef PORT_IDLE_THREAD_STACK_SIZE
#define PORT_IDLE_THREAD_STACK_SIZE     32
#endif

/**
 * @brief   Per-thread stack overhead for interrupts servicing.
 * @details This constant is used in the calculation of the correct working
 *          area size.
 *          This value can be zero on those architecture where there is a
 *          separate interrupt stack and the stack space between @p intctx and
 *          @p extctx is known to be zero.
 * @note    In this port 0x10 is a safe value, it can be reduced after careful
 *          analysis of the generated code.
 */
#ifndef PORT_INT_REQUIRED_STACK
#if defined(MIPS_USE_MIPS16_ISA)
#define PORT_INT_REQUIRED_STACK         (MIPS_STACK_FRAME_SIZE*4)
#else
#define PORT_INT_REQUIRED_STACK         (MIPS_STACK_FRAME_SIZE*2)
#endif
#endif

/**
 * @brief   Enforces a correct alignment for a stack area size value.
 */
#define STACK_ALIGN(n) ((((n) - 1) | (sizeof(stkalign_t) - 1)) + 1)

/**
 * @brief   Computes the thread working area global size.
 */
#define THD_WA_SIZE(n) STACK_ALIGN(sizeof(Thread) + \
      sizeof(struct intctx) +                       \
      sizeof(struct extctx) +                       \
      (n) + (PORT_INT_REQUIRED_STACK))

/**
 * @brief   Static working area allocation.
 * @details This macro is used to allocate a static thread working area
 *          aligned as both position and size.
 */
#define WORKING_AREA(s, n) stkalign_t s[THD_WA_SIZE(n) / sizeof(stkalign_t)]

/**
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#if !defined(PORT_IRQ_PROLOGUE)
#define PORT_IRQ_PROLOGUE() do {} while (0)
#endif /* !defined(PORT_IRQ_PROLOGUE) */

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#if !defined(PORT_IRQ_EPILOGUE)
#define PORT_IRQ_EPILOGUE() do {} while (0)
#endif /* !defined(PORT_IRQ_EPILOGUE) */

/**
 * @brief   IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#if !defined(PORT_IRQ_HANDLER)
#define PORT_IRQ_HANDLER(id) void MIPS_HW_IRQ ## id ## _handler (void)
#endif /* !defined(PORT_IRQ_HANDLER) */

/**
 * @brief   Port-related initialization code.
 * @note    This function is empty in this port.
 */
#define port_init() do {} while (0)

/**
 * @brief   Kernel-lock action.
 * @details Usually this function just disables interrupts but may perform
 *          more actions.
 * @note    In this port it disables the IRQ sources.
 */
#if defined(MIPS_USE_MIPS16_ISA)
void __nomips16 port_lock(void);
#else
#define port_lock() MIPS_DISABLE_IRQ()
#endif

/**
 * @brief   Kernel-unlock action.
 * @details Usually this function just enables interrupts but may perform
 *          more actions.
 * @note    In this port it enables both the IRQ sources.
 */
#if defined(MIPS_USE_MIPS16_ISA)
void __nomips16 port_unlock(void);
#else
#define port_unlock() MIPS_RESTORE_IRQ(1)
#endif

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @details This function is invoked before invoking I-class APIs from
 *          interrupt handlers. The implementation is architecture dependent,
 *          in its simplest form it is void.
 * @note    Empty in this port.
 */
#define port_lock_from_isr() do {} while (0)

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @details This function is invoked after invoking I-class APIs from interrupt
 *          handlers. The implementation is architecture dependent, in its
 *          simplest form it is void.
 * @note    Empty in this port.
 */
#define port_unlock_from_isr() do {} while (0)

/**
 * @brief   Disables all the interrupt sources.
 * @note    Of course non-maskable interrupt sources are not included.
 */
#if defined(MIPS_USE_MIPS16_ISA)
void __nomips16 port_disable(void);
#else
#define port_disable() MIPS_DISABLE_IRQ()
#endif

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 * @note    Interrupt sources above kernel level remains enabled.
 * @note    In this port it does nothing.
 */
#define port_suspend() do {} while (0)

/**
 * @brief   Enables all the interrupt sources.
 */
#if defined(MIPS_USE_MIPS16_ISA)
void __nomips16 port_enable(void);
#else
#define port_enable() MIPS_RESTORE_IRQ(1)
#endif

/**
 * @brief   MIPS32r2-specific wait for interrupt.
 */
#if !defined(port_wait_for_interrupt) || defined(__DOXYGEN__)
#if (MIPS_ENABLE_WFI_IDLE == FALSE) || defined(__DOXYGEN__)
#define port_wait_for_interrupt() do {} while (0)
#else
#if defined(MIPS_USE_MIPS16_ISA)
void __nomips16 port_wait_for_interrupt(void);
#else
#define port_wait_for_interrupt() MIPS_SIMPLE_ASM(wait)
#endif
#endif
#endif

/**
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 * @note    Implemented as inlined code for performance reasons.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
#if CH_DBG_ENABLE_STACK_CHECK
#define port_switch(ntp, otp) {                             \
    register struct intctx *sp ;                            \
    asm volatile ("move %0, $sp;" : "=r"(sp) : : "memory"); \
    if ((stkalign_t *)(sp - 1) < otp->p_stklimit)           \
      chDbgPanic("stack overflow");                         \
    _port_switch_mips(ntp, otp);                            \
  }
#else /* !CH_DBG_ENABLE_STACK_CHECK */
#define port_switch(ntp, otp) _port_switch_mips(ntp, otp)
#endif /* !CH_DBG_ENABLE_STACK_CHECK */

/**
 * @brief   Excludes the default @p chSchIsPreemptionRequired() implementation.
 */
#define PORT_OPTIMIZED_ISPREEMPTIONREQUIRED

#if (CH_TIME_QUANTUM > 0) || defined(__DOXYGEN__)
/**
 * @brief   Inline-able version of this kernel function.
 */
#define chSchIsPreemptionRequired()                                         \
  (currp->p_preempt ? firstprio(&rlist.r_queue) > currp->p_prio :           \
                      firstprio(&rlist.r_queue) >= currp->p_prio)
#else /* CH_TIME_QUANTUM == 0 */
#define chSchIsPreemptionRequired()                                         \
  (firstprio(&rlist.r_queue) > currp->p_prio)
#endif /* CH_TIME_QUANTUM == 0 */

#define port_reset_mips_timer() do {                      \
    c0_set_compare(c0_get_count() +                       \
        (MIPS_TIMER_FREQ + CH_FREQUENCY/2) / CH_FREQUENCY); \
} while (0)

#define port_init_mips_timer() do {                      \
    port_reset_mips_timer();                             \
    c0_set_cause(c0_get_cause()&~(1<<27));               \
} while (0)

#ifdef __cplusplus
extern "C" {
#endif
  void __nomips16 port_halt(void);
  void __nomips16 _port_switch_mips(Thread *ntp, Thread *otp);
  void __nomips16 _port_thread_start(void);
#ifdef __cplusplus
}
#endif

#endif /* _CHCORE_H_ */

/** @} */
