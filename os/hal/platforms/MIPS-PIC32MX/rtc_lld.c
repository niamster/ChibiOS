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
 * @file    MIPS-PIC32MX/rtc_lld.c
 * @brief   MIPS-PIC32MX RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

/**
 * @brief   Configuration bits of rtccon register.
 */
enum rtcConBits {
  RTC_CON_ON       = 15, /* RTC Module On */
  RTC_CON_RTSECSEL = 7,  /* 1 = RTCC Seconds Clock is selected for the RTCC pin, 0 = RTCC Alarm Pulse is selected for the RTCC pin */
  RTC_CON_WREN     = 3,  /* RTC Value Registers Write Enable */
  RTC_CON_RTCOE    = 0,  /* RTC Output Enable */
};

/**
 * @brief   Configuration bits of rtcalrm register.
 */
enum rtcAlrmBits {
  RTC_ALRM_ON      = 15, /* RTC Alarm On */
  RTC_ALRM_CHIME   = 14, /* Enable Chime */

  /* Alarm repeat period */
  RTC_ALRM_REPEAT_MASK  = 0xF,
  RTC_ALRM_REPEAT_SHIFT = 8,
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define dec2bin(d)   (10 * ((d) >> 4) + ((d) & 0x0f))
#define bin2dec(b)   ((((b) / 10) << 4) | ((b) % 10))

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   rtccon;
  PicReg   rtcalrm;
  PicReg   rtctime;
  PicReg   rtcdate;
  PicReg   alrmtime;
  PicReg   alrmdate;
} RtcPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   RTC IRQ handler.
 *
 * @param[in] data        Driver associated with RTC
 */
static void lld_serve_interrupt(uint32_t irq, void *data) {
  RTCDriver *rtcd = data;

  (void)irq;

  chSysLockFromIsr();

  chDbgAssert(rtcd->alrm_cb, "rtc_lld_serve_interrupt(), #1", "no alarm callback");

  rtcd->alrm_cb(rtcd, RTC_EVENT_ALARM);

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initialize RTC.
 *
 * @notapi
 */
void rtc_lld_init(void){
}

/**
 * @brief   RTC Driver configuration.
 *
 * @param[in] rtcd      pointer to RTC driver structure
 * @param[in] config    pointer to a @p RTCConfig structure
 *
 * @notapi
 */
void rtc_lld_config(RTCDriver *rtcd, const RTCConfig *config) {
  RtcPort *port;

  chDbgAssert(config, "rtc_lld_config(), #1", "invalid configuration");
  chDbgAssert(config->base, "rtc_lld_config(), #2", "invalid configuration");

  rtcd->base = config->base;
  rtcd->irq = config->irq;

  port = (RtcPort *)rtcd->base;

  hal_system_unlock();
  OSCCONSET = _OSCCON_SOSCEN_MASK;
  port->rtccon.reg = 1 << RTC_CON_WREN;
  hal_system_lock();

  if (config->out.enable) {
    if (RTC_EVENT_SECOND == config->out.evt)
      port->rtccon.set = 1 << RTC_CON_RTSECSEL;

    port->rtccon.set = 1 << RTC_CON_RTCOE;
  }

  port->rtccon.set = 1 << RTC_CON_ON;
  
#if HAL_USE_EIC
  eicRegisterIrq(rtcd->irq, lld_serve_interrupt, rtcd);
#endif
}

/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored. There is no possibility
 *          to change it on STM32F1xx platform.
 *
 * @param[in] rtcd      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_set_time(RTCDriver *rtcd, const RTCTime *timespec) {
  RtcPort *port = (RtcPort *)rtcd->base;

  port->rtctime.reg = bin2dec(timespec->hours) << 24
    | bin2dec(timespec->min) << 16
    | bin2dec(timespec->sec) << 8;

  port->rtcdate.reg = bin2dec(timespec->year) << 24
    | bin2dec(timespec->month) << 16
    | bin2dec(timespec->day) << 8
    | bin2dec(timespec->wday);
}

/**
 * @brief   Get current time.
 *
 * @param[in] rtcd      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_get_time(RTCDriver *rtcd, RTCTime *timespec) {
  RtcPort *port = (RtcPort *)rtcd->base;
  uint32_t date, time;

  date = port->rtcdate.reg;
  time = port->rtctime.reg;

  timespec->hours = dec2bin((time >> 24)&0xff);
  timespec->min = dec2bin((time >> 16)&0xff);
  timespec->sec = dec2bin((time >> 8)&0xff);

  timespec->year = dec2bin((date >> 24)&0xff);
  timespec->month = dec2bin((date >> 16)&0xff);
  timespec->day = dec2bin((date >> 8)&0xff);
  timespec->wday = dec2bin(date&0xff);
}

/**
 * @brief   Set alarm time.
 *
 * @note      Default value after BKP domain reset is 0xFFFFFFFF
 *
 * @param[in] rtcd      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[in] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_set_alarm(RTCDriver *rtcd,
                       rtcalarm_t alarm,
                       const RTCAlarm *alarmspec) {
  RtcPort *port = (RtcPort *)rtcd->base;

  (void)alarm;

  port->alrmtime.reg = bin2dec(alarmspec->ts.hours) << 24
    | bin2dec(alarmspec->ts.min) << 16
    | bin2dec(alarmspec->ts.sec) << 8;

  port->alrmdate.reg = bin2dec(alarmspec->ts.month) << 16
    | bin2dec(alarmspec->ts.day) << 8
    | bin2dec(alarmspec->ts.wday);

  port->rtcalrm.set = (1 << RTC_ALRM_ON)
    | ((alarmspec->period & RTC_ALRM_REPEAT_MASK) << RTC_ALRM_REPEAT_SHIFT)
    | 1;

  if (alarmspec->repeat)
    port->rtcalrm.set = 1 << RTC_ALRM_CHIME;
  else
    port->rtcalrm.clear = 1 << RTC_ALRM_CHIME;
}

/**
 * @brief   Get current alarm.
 * @note    If an alarm has not been set then the returned alarm specification
 *          is not meaningful.
 *
 * @note    Default value after BKP domain reset is 0xFFFFFFFF.
 *
 * @param[in] rtcd      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcd,
                       rtcalarm_t alarm,
                       RTCAlarm *alarmspec) {
  RtcPort *port = (RtcPort *)rtcd->base;
  uint32_t date, time;
  uint32_t alrm;

  (void)alarm;

  date = port->alrmdate.reg;
  time = port->alrmtime.reg;
  alrm = port->rtcalrm.reg;

  alarmspec->ts.hours = dec2bin((time >> 24)&0xff);
  alarmspec->ts.min = dec2bin((time >> 16)&0xff);
  alarmspec->ts.sec = dec2bin((time >> 8)&0xff);

  alarmspec->ts.month = dec2bin((date >> 16)&0xff);
  alarmspec->ts.day = dec2bin((date >> 8)&0xff);
  alarmspec->ts.wday = dec2bin(date&0xff);

  if (alrm & (1 << RTC_ALRM_CHIME))
    alarmspec->repeat = TRUE;
  else
    alarmspec->repeat = FALSE;

  alarmspec->period = (alrm >> RTC_ALRM_REPEAT_SHIFT) & RTC_ALRM_REPEAT_MASK;
}

/**
 * @brief   Enables or disables RTC callbacks.
 * @details This function enables or disables callbacks, use a @p NULL pointer
 *          in order to disable a callback.
 *
 * @param[in] rtcd      pointer to RTC driver structure
 * @param[in] callback  callback function pointer or @p NULL
 *
 * @notapi
 */
void rtc_lld_set_callback(RTCDriver *rtcd, rtccb_t callback) {
  rtcd->alrm_cb = callback;

  if (callback) {
#if HAL_USE_EIC
    eicEnableIrq(rtcd->irq);
#endif
  } else {
#if HAL_USE_EIC
    eicAckIrq(rtcd->irq);
    eicDisableIrq(rtcd->irq);
#endif
  }
}

#endif /* HAL_USE_RTC */

/** @} */
