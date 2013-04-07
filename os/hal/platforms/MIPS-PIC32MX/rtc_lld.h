/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

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
 * @file    MIPS-PIC32MX/rtc_lld.h
 * @brief   MIPS-PIC32MX RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#ifndef _RTC_LLD_H_
#define _RTC_LLD_H_

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   This RTC implementation supports callbacks.
 */
#define RTC_SUPPORTS_CALLBACKS      TRUE

/**
 * @brief   One alarm comparator available.
 */
#define RTC_ALARMS                  1

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an RTC alarm time stamp.
 */
typedef struct RTCAlarm RTCAlarm;

/**
 * @brief   Type of an RTC alarm.
 * @details Meaningful on platforms with more than 1 alarm comparator.
 */
typedef uint32_t rtcalarm_t;

/**
 * @brief   Type of an RTC event.
 */
typedef enum {
  RTC_EVENT_SECOND = 0,                 /** Triggered every second.         */
  RTC_EVENT_ALARM = 1,                  /** Triggered on alarm.             */
} rtcevent_t;

/**
 * @brief   Type of a generic RTC callback.
 */
typedef void (*rtccb_t)(RTCDriver *rtcd, rtcevent_t event);

/**
 * @brief   Structure representing an RTC time stamp.
 */
struct RTCTime {
  /**
   * @brief Year.
   */
  uint8_t  year;
  /**
   * @brief Month.
   */
  uint8_t  month;
  /**
   * @brief Day.
   */
  uint8_t  day;
  /**
   * @brief Weekday.
   */
  uint8_t  wday;
  /**
   * @brief Hours.
   */
  uint8_t  hours;
  /**
   * @brief Minutes.
   */
  uint8_t  min;
  /**
   * @brief Seconds.
   */
  uint8_t  sec;
};

typedef enum {
  ALARM_PERIOD_HALF_SECOND,
  ALARM_PERIOD_SECOND,
  ALARM_PERIOD_10_SECONDS,
  ALARM_PERIOD_MINUTE,
  ALARM_PERIOD_10_MINUTES,
  ALARM_PERIOD_HOUR,
  ALARM_PERIOD_DAY,
  ALARM_PERIOD_WEEK,
  ALARM_PERIOD_MONTH,
  ALARM_PERIOD_YEAR,
} alarmperiod_t;

/**
 * @brief   Structure representing an RTC alarm time stamp.
 */
struct RTCAlarm {
  /**
   * @brief Alart time and date.
   */
  struct {
    /**
     * @brief Month.
     */
    uint8_t  month;
    /**
     * @brief Day.
     */
    uint8_t  day;
    /**
     * @brief Weekday.
     */
    uint8_t  wday;
    /**
     * @brief Hours.
     */
    uint8_t  hours;
    /**
     * @brief Minutes.
     */
    uint8_t  min;
    /**
     * @brief Seconds.
     */
    uint8_t  sec;
  } ts;
  /**
   * @brief Enable alarm repeat.
   */
  bool_t          repeat;
  /**
   * @brief Alarm repeat period
   */
  alarmperiod_t   period;
};

/**
 * @brief   Structure representing an RTC driver configuration.
 */
struct RTCConfig {
  struct {
    /**
     * @brief Enable output on RTCC pin.
     */
    bool_t                enable;
    /**
     * @brief Event to trigger output on RTCC pin.
     */
    rtcevent_t            evt;
  } out;
  /**
   * @brief RTC interrupt.
   */
  uint8_t               irq;
  /**
   * @brief RTC port.
   */
  uint32_t              base;
};

/**
 * @brief   Structure representing an RTC driver.
 */
struct RTCDriver {
  /**
   * @brief Alarm callback.
   */
  rtccb_t           alrm_cb;
  /**
   * @brief RTC interrupt.
   */
  uint8_t               irq;
  /**
   * @brief RTC port.
   */
  uint32_t              base;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void rtc_lld_init(void);
  void rtc_lld_config(RTCDriver *rtcd, const RTCConfig *config);
  void rtc_lld_set_time(RTCDriver *rtcd, const RTCTime *timespec);
  void rtc_lld_get_time(RTCDriver *rtcd, RTCTime *timespec);
  void rtc_lld_set_alarm(RTCDriver *rtcd,
                         rtcalarm_t alarm,
                         const RTCAlarm *alarmspec);
  void rtc_lld_get_alarm(RTCDriver *rtcd,
                         rtcalarm_t alarm,
                         RTCAlarm *alarmspec);
  void rtc_lld_set_callback(RTCDriver *rtcd, rtccb_t callback);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_RTC */

#endif /* _RTC_LLD_H_ */

/** @} */
