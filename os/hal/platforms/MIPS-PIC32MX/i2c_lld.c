/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
   Concepts and parts of this file have been contributed by Dmytro Milinevskyy <milinevskyy@gmail.com>
 */

/**
 * @file    MIPS-PIC32MX/i2c_lld.c
 * @brief   I2C Driver subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local macros.                                                      */
/*===========================================================================*/

/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] i2cd      pointer to the @p I2CDriver object
 * @param[in] msg       wakeup message
 *
 * @notapi
 */
#define wakeup_isr(i2cd, msg) {                                             \
  chSysLockFromIsr();                                                       \
  if ((i2cd)->thread != NULL) {                                             \
    Thread *tp = (i2cd)->thread;                                            \
    (i2cd)->thread = NULL;                                                  \
    tp->p_u.rdymsg = (msg);                                                 \
    chSchReadyI(tp);                                                        \
  }                                                                         \
  chSysUnlockFromIsr();                                                     \
}

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Configuration bits of con register.
 */
enum i2cConBits {
  I2C_CON_ON      = 15, /* I2C Module On */
  I2C_CON_ACKDT   = 5,  /* Acknowledge Data (1: NACK is sent, 0: ACK is sent) */
  I2C_CON_ACKEN   = 4,  /* Acknowledge Sequence Enable */
  I2C_CON_RCEN    = 3,  /* Receive Enable */
  I2C_CON_PEN     = 2,  /* Stop Condition Enable */
  I2C_CON_RSEN    = 1,  /* Restart Condition Enable */
  I2C_CON_SEN     = 0,  /* Start Condition Enable */
};

/**
 * @brief   Configuration bits of stat register.
 */
enum i2cStatBits {
  I2C_STAT_ACKSTAT = 15, /* Acknowledge Status */
  I2C_STAT_TRSTAT  = 14, /* Transmit Status */
  I2C_STAT_BCL     = 10, /* Master Bus Collision Detect */
  I2C_STAT_IWCOL   = 7,  /* Write Collision Detect */
  I2C_STAT_I2COV   = 6,  /* Receive Overflow Status */
  I2C_STAT_P       = 4,  /* Stop (indicates that a Stop bit has been detected last) */
  I2C_STAT_S       = 3,  /* Start (indicates that a Start(or Restart) bit has been detected last) */
  I2C_STAT_RBF     = 1,  /* Receive complete */
  I2C_STAT_TBF     = 0,  /* Transmit in progress */
};

/**
 * @brief   States of internal FSM.
 */
enum i2cFsm {
  I2C_FSM_START,
  I2C_FSM_ADDRESS,
  I2C_FSM_TX,
  I2C_FSM_RX,
  I2C_FSM_RESTART,
  I2C_FSM_ACK,
  I2C_FSM_STOP,
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   con;
  PicReg   stat;
  PicReg   add;
  PicReg   msk;
  PicReg   brg;
  PicReg   trn;
  uint32_t rcv;
} I2CPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void
i2c_reset(I2CPort *port) {
  port->con.clear = 1 << I2C_CON_ON;
  port->con.set = 1 << I2C_CON_ON;
}

static void
i2c_start(I2CPort *port) {
  port->con.set = 1 << I2C_CON_SEN;
}

static void
i2c_stop(I2CPort *port) {
  port->con.set = 1 << I2C_CON_PEN;
}

static void
i2c_restart(I2CPort *port) {
  port->con.set = 1 << I2C_CON_RSEN;
}

static void
i2c_receive_enable(I2CPort *port) {
  port->con.set = 1 << I2C_CON_RCEN;
}

static void
i2c_transmit(I2CPort *port, uint8_t data) {
  chDbgAssert(!(port->stat.reg & (1 << I2C_STAT_TBF)), "i2c_transmit(), #1", "");

  port->trn.reg = data;
}

static void
i2c_receive(I2CPort *port, uint8_t *data) {
  chDbgAssert(port->stat.reg & (1 << I2C_STAT_RBF), "i2c_receive(), #1", "");

  *data = port->rcv;
}

static void
i2c_ack(I2CPort *port, bool_t ack) {
  if (ack)
    port->con.clear = 1 << I2C_CON_ACKDT;
  else
    port->con.set = 1 << I2C_CON_ACKDT;

  port->con.set = 1 << I2C_CON_ACKEN;
}

static void
i2c_lld_receive(I2CDriver *i2cd) {
  const I2CConfig *cfg = i2cd->config;
  I2CPort *port = (I2CPort *)cfg->base;

  i2c_receive(port, i2cd->rxbuf);

  ++i2cd->rxbuf;
  --i2cd->rxbytes;
}

static void
i2c_lld_transmit(I2CDriver *i2cd) {
  const I2CConfig *cfg = i2cd->config;
  I2CPort *port = (I2CPort *)cfg->base;

  i2c_transmit(port, *i2cd->txbuf);

  ++i2cd->txbuf;
  --i2cd->txbytes;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   I2C IRQ handler.
 *
 * @param[in] data        Driver associated with I2C
 */
static void lld_serve_interrupt(uint32_t irq, void *data) {
  I2CDriver *i2cd = data;
  const I2CConfig *cfg = i2cd->config;
  I2CPort *port = (I2CPort *)cfg->base;
  const uint32_t stat = port->stat.reg;

  chSysLockFromIsr();

  if (cfg->bus_irq == irq) {
    chDbgAssert(stat & (1 << I2C_STAT_BCL), "I2C IRQ, #1", "");
    i2cd->errors = I2CD_BUS_ERROR;
    wakeup_isr(i2cd, RDY_RESET);
  } else if (cfg->master_irq == irq) {
    chDbgAssert(!(port->con.reg &
            ( (1 << I2C_CON_ACKEN)
                | (1 << I2C_CON_RCEN)
                | (1 << I2C_CON_PEN)
                | (1 << I2C_CON_RSEN)
                | (1 << I2C_CON_SEN) )
            ), "I2C IRQ, #2", "");

    if (stat & ((1 << I2C_STAT_BCL)
            /* | (1 << I2C_STAT_I2COV) */
            /* | (1 << I2C_STAT_IWCOL) */)) {
      i2cd->errors = I2CD_BUS_ERROR;
      wakeup_isr(i2cd, RDY_RESET);
      chSysUnlockFromIsr();
      return;
    }

    switch (i2cd->fsm) {
      case I2C_FSM_START:
        /* chDbgAssert(stat & (1 << I2C_STAT_S), "I2C IRQ, #3", ""); */
        /* chDbgAssert(!(stat & (1 << I2C_STAT_P)), "I2C IRQ, #4", ""); */

        i2c_transmit(port, i2cd->addr);
        i2cd->fsm = I2C_FSM_ADDRESS;
        break;
      case I2C_FSM_ADDRESS:
        if (stat & (1 << I2C_STAT_ACKSTAT)) {
          i2cd->errors = I2CD_ACK_FAILURE;
          wakeup_isr(i2cd, RDY_RESET);
          break;
        }
        if (i2cd->txbytes) {
          i2c_lld_transmit(i2cd);
          i2cd->fsm = I2C_FSM_TX;
        } else {
          i2c_receive_enable(port);
          i2cd->fsm = I2C_FSM_RX;
        }
        break;
      case I2C_FSM_TX:
        if (stat & (1 << I2C_STAT_ACKSTAT)) {
          i2cd->errors = I2CD_ACK_FAILURE;
          wakeup_isr(i2cd, RDY_RESET);
          break;
        }
        if (i2cd->txbytes)
          i2c_lld_transmit(i2cd);
        else if (i2cd->rxbytes) {
          i2c_restart(port);
          i2cd->fsm = I2C_FSM_RESTART;
        } else {
          i2c_stop(port);
          i2cd->fsm = I2C_FSM_STOP;
        }
        break;
      case I2C_FSM_RX:
        i2c_lld_receive(i2cd);
        i2c_ack(port, i2cd->rxbytes?TRUE:FALSE);
        i2cd->fsm = I2C_FSM_ACK;
        break;
      case I2C_FSM_RESTART:
        i2c_transmit(port, i2cd->addr|1);
        i2cd->fsm = I2C_FSM_ADDRESS;
        break;
      case I2C_FSM_ACK:
        if (i2cd->rxbytes) {
          i2c_receive_enable(port);
          i2cd->fsm = I2C_FSM_RX;
        } else {
          i2c_stop(port);
          i2cd->fsm = I2C_FSM_STOP;
        }
        break;
      case I2C_FSM_STOP:
        /* chDbgAssert(!(stat & (1 << I2C_STAT_S)), "I2C IRQ, #5", ""); */
        /* chDbgAssert(stat & (1 << I2C_STAT_P), "I2C IRQ, #6", ""); */

        wakeup_isr(i2cd, RDY_OK);
        break;
    }
  } else if (cfg->slave_irq == irq) {
    chDbgPanic("I2C slave IRQ");
  } else
    chDbgPanic("Unknown I2C IRQ");

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cd      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cd) {
  if (i2cd->state == I2C_STOP) {
    const I2CConfig *cfg = i2cd->config;
    I2CPort *port = (I2CPort *)cfg->base;

    port->brg.reg = (hal_pb_frequency()/cfg->frequency)/2 - 2;

    port->con.set = 1 << I2C_CON_ON;

#if HAL_USE_EIC
    eicRegisterIrq(cfg->bus_irq, lld_serve_interrupt, i2cd);
    eicEnableIrq(cfg->bus_irq);

    /* eicRegisterIrq(cfg->slave_irq, lld_serve_interrupt, i2cd); */
    /* eicEnableIrq(cfg->slave_irq); */

    eicRegisterIrq(cfg->master_irq, lld_serve_interrupt, i2cd);
    eicEnableIrq(cfg->master_irq);
#endif
  }
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cd      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cd) {
  if (i2cd->state != I2C_STOP) {
    const I2CConfig *cfg = i2cd->config;
    I2CPort *port = (I2CPort *)cfg->base;

    port->con.clear = 1 << I2C_CON_ON;

#if HAL_USE_EIC
    eicDisableIrq(cfg->bus_irq);
    eicUnregisterIrq(cfg->bus_irq);

    /* eicDisableIrq(cfg->slave_irq); */
    /* eicUnregisterIrq(cfg->slave_irq); */

    eicDisableIrq(cfg->master_irq);
    eicUnregisterIrq(cfg->master_irq);
#endif
  }
}

/**
 * @brief   Transmits data via the I2C bus as master.
 *
 * @param[in] i2cd      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cd, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {
    const I2CConfig *cfg = i2cd->config;
    I2CPort *port = (I2CPort *)cfg->base;

    chDbgAssert(txbuf || rxbuf, "i2c_lld_master_transmit_timeout(), #1", "");

    i2cd->addr = addr << 1;

    i2cd->txbuf   = txbuf;
    i2cd->txbytes = txbytes;
    i2cd->rxbuf   = rxbuf;
    i2cd->rxbytes = rxbytes;

    i2c_start(port);
    i2cd->fsm = I2C_FSM_START;

    /* Waits for the operation completion.*/
    i2cd->thread = chThdSelf();
    chThdSleepS(timeout);

    if (RDY_OK != chThdSelf()->p_u.rdymsg)
      i2c_reset(port);

    return chThdSelf()->p_u.rdymsg;
}

#endif /* HAL_USE_I2C */

/** @} */
