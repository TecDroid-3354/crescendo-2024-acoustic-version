
// Tejuino Board functions V1
// Author: IOT4U Technology
// https://www.iot4utechnology.com
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@////////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@///////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@//////////////////////////////////////////////////////////@@@@@@@@@@
// @@@@@@@@/////////////////////////////////////////////////////////////////@@@@@@@
// @@@@@@@///////////////////////////////////////////////////////////////////@@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@///////////////@@@@@@@@////////////////////////@@@@@@@@//////////////@@@@@
// @@@@@@//////////////@@@@@@@@@@@////////////////////@@@@@@@@@@@#////////////@@@@@
// @@@@@@/////////////@@@@@@@@@@@@////////////////////@@@@@@@@@@@@////////////@@@@@
// @@@@@@/////////////*@@@@@@@@@@@////////////////////@@@@@@@@@@@/////////////@@@@@
// @@@@@@////////////////@@@@@@@////////////////////////@@@@@@@*//////////////@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@/////////////////////////////////////////////////////////////////////@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@///////////////////////////////////////////////@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@/////////////////////////////////////////////@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@/////////////////////////////////@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/* ============================================
 Tejuino Board source code is placed under the MIT license
 Copyright (c) 2024 IOT4U Technology https://www.iot4utechnology.com 

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */
//Compiled for WPILib 2024.2.1

//IMPORTANT:  Experimental version no tested fully on robot- Use at your own risk

#pragma once

#include <fmt/core.h>
#include "hal/CANAPI.h"

class TejuinoBoard  {

public:
  int TEJUINO_DEVICE_NUMBER_0 = 0;
  int TEJUINO_DEVICE_NUMBER_1 = 1;

  // Led strip names
  int LED_STRIP_0 = 0;
  int LED_STRIP_1 = 1;
  int LED_STRIP_2 = 2;

  void init(int tejuino_device_number);
  void single_led_control(int led_strip, int led_number, int red, int green, int blue);
  void all_led_control(int led_strip, int red, int green, int blue);
  void all_leds_red(int led_strip);
  void all_leds_green(int led_strip);
  void all_leds_blue(int led_strip);
  void all_leds_white(int led_strip);
  void all_leds_purple(int led_strip);
  void all_leds_yellow(int led_strip);
  void turn_off_all_leds(int led_strip);
  void rainbow_effect(int led_strip);
  void stingbot_effect(int led_strip);
  void escuderia_effect(int led_strip);

};
