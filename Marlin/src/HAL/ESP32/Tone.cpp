/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * Copypaste of SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
<<<<<<< HEAD:Marlin/src/feature/repeat.h
#pragma once

#include "../inc/MarlinConfigPre.h"
#include "../gcode/parser.h"

#include <stdint.h>

#define MAX_REPEAT_NESTING 10

typedef struct {
  uint32_t sdpos;   // The repeat file position
  int16_t counter;  // The counter for looping
} repeat_marker_t;

class Repeat {
private:
  static repeat_marker_t marker[MAX_REPEAT_NESTING];
  static uint8_t index;
public:
  static inline void reset() { index = 0; }
  static inline bool is_active() {
    LOOP_L_N(i, index) if (marker[i].counter) return true;
    return false;
  }
  static bool is_command_M808(char * const cmd) { return cmd[0] == 'M' && cmd[1] == '8' && cmd[2] == '0' && cmd[3] == '8' && !NUMERIC(cmd[4]); }
  static void early_parse_M808(char * const cmd);
  static void add_marker(const uint32_t sdpos, const uint16_t count);
  static void loop();
  static void cancel();
};

extern Repeat repeat;
=======

/**
 * Description: Tone function for ESP32
 * Derived from https://forum.arduino.cc/index.php?topic=136500.msg2903012#msg2903012
 */

#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfig.h"
#include "HAL.h"

static pin_t tone_pin;
volatile static int32_t toggles;

void tone(const pin_t _pin, const unsigned int frequency, const unsigned long duration) {
  tone_pin = _pin;
  toggles = 2 * frequency * duration / 1000;
  HAL_timer_start(TONE_TIMER_NUM, 2 * frequency);
}

void noTone(const pin_t _pin) {
  HAL_timer_disable_interrupt(TONE_TIMER_NUM);
  WRITE(_pin, LOW);
}

HAL_TONE_TIMER_ISR() {
  HAL_timer_isr_prologue(TONE_TIMER_NUM);

  if (toggles) {
    toggles--;
    TOGGLE(tone_pin);
  }
  else noTone(tone_pin);                         // turn off interrupt
}

#endif // ARDUINO_ARCH_ESP32
>>>>>>> bugfix-2.0.x:Marlin/src/HAL/ESP32/Tone.cpp
