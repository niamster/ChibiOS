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
 * @file    MIPS-PIC32MX/adc_lld.c
 * @brief   MIPS-PIC32MX ADC Driver subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

/**
 * @brief   Configuration bits of ad1con1 register.
 */
enum adcCon1Bits {
  ADC_CON1_ON       = 15, /* ADC Module On */
  ADC_CON1_CLRASAM  = 4,  /* Stop Conversion Sequence (when the first A/D converter interrupt is generated) */
  ADC_CON1_ASAM     = 2,  /* ADC Sample Auto-Start (Sampling begins immediately after last conversion completes; SAMP bit is automatically set) */
  ADC_CON1_SAMP     = 1,  /* ADC Sample Enable */
  
  /* Data output format */
  ADC_CON1_FORM_UINT16 = 0,   /* Integer 16-bit (DOUT = 0000 0000 0000 0000 0000 00dd dddd dddd) */
  ADC_CON1_FORM_MASK   = 0x7,
  ADC_CON1_FORM_SHIFT  = 8,

  /* Convertion trigger source */
  ADC_CON1_SSRC_AUTO   = 7,   /* Internal counter ends sampling and starts conversion (auto convert) */
  ADC_CON1_SSRC_MANUAL = 0,   /* Clearing SAMP bit ends sampling and starts conversion */
  ADC_CON1_SSRC_MASK   = 0x7,
  ADC_CON1_SSRC_SHIFT  = 5,
};

/**
 * @brief   Configuration bits of ad1con2 register.
 */
enum adcCon2Bits {
  /* Voltage Reference Configuration */
  ADC_CON2_VCFG_MASK   = 0x3,
  ADC_CON2_VCFG_SHIFT  = 13,

  /* Sample/Convert Sequences Per Interrupt */
  ADC_CON2_SMPI_MASK   = 0xf,
  ADC_CON2_SMPI_SHIFT  = 2,
};

/**
 * @brief   Configuration bits of ad1con3 register.
 */
enum adcCon3Bits {
  ADC_CON3_ADRC     = 15, /* ADC Conversion Clock Source (1: ADC internal RC clock; 0: clock derived from Peripheral Bus Clock (PBclock)) */

  /* Auto-Sample Time */
  ADC_CON3_SAMC_MASK   = 0x1f,
  ADC_CON3_SAMC_SHIFT  = 8,
};

/**
 * @brief   Configuration bits of ad1chs register.
 */
enum adcChsBits {
  /* Positive Input Select bits for MUX A Multiplexer Setting */
  ADC_CHS_SA_MASK   = 0xf,
  ADC_CHS_SA_SHIFT  = 16,
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   ad1con1;
  PicReg   ad1con2;
  PicReg   ad1con3;
  PicReg   pad0;
  PicReg   ad1chs;
  PicReg   ad1cssl;
  PicReg   ad1pcfg;
  PicReg   ad1buf[16];
} AdcPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint8_t adc_channel_index(adc_channels_t channels) {
  uint8_t ch;

  for (ch=0;ch<16;++ch)
    if (channels & (1 << ch))
      return ch;

  /* should not be reachable */
  
  return 0;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   ADC IRQ handler.
 *
 * @param[in] data        Driver associated with ADC
 */
static void lld_serve_interrupt(uint32_t irq, void *data) {
  ADCDriver *adcd = data;
  const ADCConfig *cfg = adcd->config;
  AdcPort *port = (AdcPort *)cfg->base;
  const ADCConversionGroup *grpp = adcd->grpp;
  adcsample_t *samples;
  uint8_t i;

  (void)irq;

  chSysLockFromIsr();

  i = 0;

  if (adcd->samples) {
    samples = adcd->samples + adcd->current * adcd->depth;
    for (;i<adcd->depth;++i)
      samples[i] = port->ad1buf[i].reg;
  }

  /* All samples must be read from the result registers to clear the interrupt source. */
  for (;i<16;++i) {
    volatile adcsample_t dummy = port->ad1buf[i].reg;
    (void)dummy;
  }

#if HAL_USE_EIC
  eicAckIrq(cfg->irq);
#endif

  if (!grpp) // if was stopped
    return;

  if (++adcd->current == grpp->num_channels) {
    _adc_isr_full_code(adcd);

    if (grpp->circular) {
      uint8_t channel;

      channel = adc_channel_index(grpp->channels);

      adcd->current = 0;
      adcd->channels = grpp->channels & ~(1 << channel);

      port->ad1con1.clear = 1 << ADC_CON1_ON;

      port->ad1chs.clear = ADC_CHS_SA_MASK << ADC_CHS_SA_SHIFT;
      port->ad1chs.set = channel << ADC_CHS_SA_SHIFT;

      port->ad1con1.set = (1 << ADC_CON1_ASAM) | (1 << ADC_CON1_SAMP);

      port->ad1con1.set = 1 << ADC_CON1_ON;
    }
  } else {
    uint8_t channel;

    channel = adc_channel_index(adcd->channels);

    adcd->channels &= ~(1 << channel);

    port->ad1con1.clear = 1 << ADC_CON1_ON;

    port->ad1chs.clear = ADC_CHS_SA_MASK << ADC_CHS_SA_SHIFT;
    port->ad1chs.set = channel << ADC_CHS_SA_SHIFT;

    port->ad1con1.set = (1 << ADC_CON1_ASAM) | (1 << ADC_CON1_SAMP);

    port->ad1con1.set = 1 << ADC_CON1_ON;
  }

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcd      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcd) {
  AdcPort *port;
  const ADCConfig *cfg;

  if (adcd->state != ADC_STOP)
    return;

  cfg = adcd->config;
  port = (AdcPort *)cfg->base;

  chDbgAssert(port, "adc_lld_start(), #1", "wrong configuration");
  chDbgAssert(cfg->stime > 0 && cfg->stime <= 31, "adc_lld_start(), #2", "wrong configuration");

  port->ad1con1.reg = (1 << ADC_CON1_CLRASAM)
    | ((ADC_CON1_FORM_UINT16 & ADC_CON1_FORM_MASK) << ADC_CON1_FORM_SHIFT)
    | ((ADC_CON1_SSRC_AUTO & ADC_CON1_SSRC_MASK) << ADC_CON1_SSRC_SHIFT);

  port->ad1con2.reg = (cfg->vref & ADC_CON2_VCFG_MASK) << ADC_CON2_VCFG_SHIFT;

  port->ad1con3.reg = (1 << ADC_CON3_ADRC) | (cfg->stime << ADC_CON3_SAMC_SHIFT);

#if HAL_USE_EIC
  eicRegisterIrq(cfg->irq, lld_serve_interrupt, adcd);
  eicEnableIrq(cfg->irq);
#endif
}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcd      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcd) {
  const ADCConfig *cfg;

  if (adcd->state != ADC_READY)
    return;

  cfg = adcd->config;

#if HAL_USE_EIC
  eicDisableIrq(cfg->irq);
  eicUnregisterIrq(cfg->irq);
#endif
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcd      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start_conversion(ADCDriver *adcd) {
  const ADCConfig *cfg = adcd->config;
  AdcPort *port = (AdcPort *)cfg->base;
  const ADCConversionGroup *grpp = adcd->grpp;
  uint8_t channel;

  chDbgAssert(adcd->depth <= 16, "adc_lld_start_conversion(), #1", "wrong number of samples");
  chDbgAssert(grpp->channels > 0, "adc_lld_start_conversion(), #2", "wrong channels mask");

  channel = adc_channel_index(grpp->channels);

  adcd->current = 0;
  adcd->channels = grpp->channels & ~(1 << channel);

  port->ad1chs.clear = ADC_CHS_SA_MASK << ADC_CHS_SA_SHIFT;
  port->ad1chs.set = channel << ADC_CHS_SA_SHIFT;

  port->ad1con2.clear = ADC_CON2_SMPI_MASK << ADC_CON2_SMPI_SHIFT;
  port->ad1con2.set = (adcd->depth - 1) << ADC_CON2_SMPI_SHIFT;

  port->ad1con1.set = (1 << ADC_CON1_ASAM) | (1 << ADC_CON1_SAMP);

  port->ad1con1.set = 1 << ADC_CON1_ON;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcd      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcd) {
  const ADCConfig *cfg = adcd->config;
  AdcPort *port = (AdcPort *)cfg->base;

  port->ad1con1.clear = 1 << ADC_CON1_ON;
}

#endif /* HAL_USE_ADC */

/** @} */
