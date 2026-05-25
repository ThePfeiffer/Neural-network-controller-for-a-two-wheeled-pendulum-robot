/***************************************************************************
 *   Copyright (C) 2022 by DTU                                             *
 *   jcan@dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a teensy 3.1 72MHz ARM processor MK20DX256 - or any higher,  *
 *   intended for 31300/1 Linear control 1
 *
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H

#define mainh "$Id: main.h 1772 2026-04-25 06:54:23Z jcan $"


#define MISSING_SYSCALL_NAMES

#include <core_pins.h>

#define REGBOT_HW41


#define PIN_LED_STATUS_6        0 // on version HW 6.0 and 6.3
//#define PIN_LED_DEBUG           13 // (LED_BUILTIN)

#define PIN_LED_STATUS          11
// #define PIN_LED_DEBUG           13 // (LED_BUILTIN)
#define PIN_START_BUTTON        37
#define PIN_LINE_LED_HIGH       12
#define PIN_LINE_LED_HIGH_6      6
#define PIN_LINE_LED_LOW        33
#define PIN_POWER_IR            36
#define PIN_POWER_ROBOT         1
#define PIN_IR_RAW_1            A15
#define PIN_IR_RAW_2            A16
#define PIN_BATTERY_VOLTAGE     A17
#define PIN_LEFT_MOTOR_CURRENT  A1
#define PIN_RIGHT_MOTOR_CURRENT A0
//                                  // version 6.3 board          power distibute (V3) function
#define PIN_LINE_SENSOR_0       A6  // Teensy pin 20;  IDC pin 3;  on (input) - motor voltage  39k/4.7k
#define PIN_LINE_SENSOR_1       A13 // Teensy pin 27;  IDC pin 4;  off (output) - low = power off after 5-7 seconds
#define PIN_LINE_SENSOR_2       A7  // Teensy pin 21;  IDC pin 5;  Off when not to IDC pin 10 (output low)
#define PIN_LINE_SENSOR_3       A12 // Teensy pin 26;  IDC pin 6;  Alive LED (high to on)
#define PIN_LINE_SENSOR_4       A8  // Teensy pin 22;  IDC pin 7;  Tb (on board - not mounted)
#define PIN_LINE_SENSOR_5       A11 // Teensy pin 25;  IDC pin 8;  T1 (Temp sensor T1 (J10))
#define PIN_LINE_SENSOR_6       A9  // Teensy pin 23;  IDC pin 9;  T2 (Temp sensor T2 (J12))
#define PIN_LINE_SENSOR_7       A10 // Teensy pin 24;  IDC pin 10; Off, when not to IDC pin 5 (input pull high)

// Motor Controller pins
#define PIN_LEFT_DIR            2
#define PIN_LEFT_PWM            3
#define PIN_RIGHT_PWM           5
#define PIN_RIGHT_DIR           4
#define PIN_LEFT_FAULT          38
#define PIN_RIGHT_FAULT         32
#define M1ENC_A         29
#define M1ENC_B         28
#define M2ENC_A         31
#define M2ENC_B         30

#define PIN_SERIAL8_TX 35
#define PIN_SERIAL8_RX 34
// Servo pins
// #ifdef REGBOT_HW41
#define PIN_SERVO1      10
#define PIN_SERVO2       9
#define PIN_SERVO3       8
#define PIN_SERVO4       7

// Global variables

extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt; /// sample time count - typically ms (not assumed to overflow)
extern volatile uint32_t tsec;  /// time that will not overrun
extern volatile uint32_t tusec; /// time that will stay within [0...999999]
// extern float missionTime;
extern const char * versioncpp;

#endif
