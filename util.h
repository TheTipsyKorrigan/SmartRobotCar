/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
// Watchdog-safe millisecond delay: resets the WDT before blocking so long
// delays in initialisation do not trigger an unintended reset.
#pragma once
#include <avr/wdt.h>
#include <Arduino.h>

inline void delay_wdt(uint16_t ms) {
  wdt_reset();
  for (unsigned long i = 0; i < ms; i++) delay(1);
}
