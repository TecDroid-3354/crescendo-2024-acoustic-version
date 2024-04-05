
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
// Compiled for WPILib 2024.2.1

// IMPORTANT:  Experimental version no tested fully on robot - Use at your own risk

#include <fmt/core.h>
#include "hal/CANAPI.h"

#include "TejuinoBoard.hh"

int TEJUINO_LED_API_ID  = 0;
int TEJUINO_GPIO_API_ID = 1; // Reserved for V2

HAL_CANDeviceType   TEJUINO_DEVICE_TYPE  = HAL_CAN_Dev_kMiscellaneous;
HAL_CANManufacturer TEJUINO_MANUFACTURER = HAL_CAN_Man_kTeamUse;

HAL_CANHandle can_handle;

////////////////////////////////////////////////////////////////
void
TejuinoBoard::init(int tejuino_device_number) {
    int status; // status not used on this experimental version

    can_handle = HAL_InitializeCAN(TEJUINO_MANUFACTURER, tejuino_device_number, TEJUINO_DEVICE_TYPE, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::single_led_control(int led_strip, int led_number, int red, int green, int blue) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = led_number;
    data[1] = 0;
    data[2] = red;
    data[3] = green;
    data[4] = blue;
    data[5] = 0;
    data[6] = 0;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////

void
TejuinoBoard::all_led_control(int led_strip, int red, int green, int blue) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = red;
    data[3] = green;
    data[4] = blue;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::all_leds_red(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 255;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::all_leds_green(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 255;
    data[4] = 0;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::all_leds_blue(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 255;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::all_leds_white(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 255;
    data[3] = 255;
    data[4] = 255;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::all_leds_purple(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 153;
    data[3] = 51;
    data[4] = 255;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::all_leds_yellow(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 255;
    data[3] = 255;
    data[4] = 0;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

void
TejuinoBoard::turn_off_all_leds(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 1;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::rainbow_effect(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 1;
    data[6] = 0;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::stingbot_effect(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 2;
    data[6] = 0;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}

////////////////////////////////////////////////////////////////
void
TejuinoBoard::escuderia_effect(int led_strip) {
    int     status; // status not used on this experimental version
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 3;
    data[6] = 0;
    data[7] = led_strip;

    HAL_WriteCANPacket(can_handle, data, 8, TEJUINO_LED_API_ID, &status);
}
