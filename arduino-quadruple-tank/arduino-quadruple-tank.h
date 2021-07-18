 /* arduino-quadruple-tank.h
 * Header file of Arduino Code of the Quadruple Tank Process for the
 * interface with Simulink via Serial Communication.
 * Authors: Leonardo Pedroso, Pedro Batista
 * Institute for Systems and Robotics, Instituto Superior Técnico, 
 * Lisbon, Portugal*/
 
 /* Developed for project DECENTER (https://decenter2021.github.io/) under MIT license
  * Copyright (c) 2021 DECENTER
  * Permission is hereby granted, free of charge, to any person obtaining 
  * a copy of this software and associated documentation files (the 
  * “Software”), to deal in the Software without restriction, including 
  * without limitation the rights to use, copy, modify, merge, publish, 
  * distribute, sublicense, and/or sell copies of the Software, and to 
  * permit persons to whom the Software is furnished to do so, subject 
  * to the following conditions:
  * The above copyright notice and this permission notice shall be included 
  * in all copies or substantial portions of the Software.
  * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS 
  * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
  * IN THE SOFTWARE.
  */

// Include open source I2C interface for reading and writing to registers directly
#include "I2Cdev.h" 

// ---------- Define ADC addresses and configuration words ----------
// Check pp. 16-22 of the datasheet of ADS1115 available in the Datasheets folder
/* Write config register (R0x01) - Configure conversion properties and input
 * 15                  / 14:12 / 11:9          / 8                         / 7:5               / 4                         / 3                         / 2                         / 1:0
 * Start: single shot  / AX    / PGA: 6.1444V  / Operating: single shot    / Data rate: 128SPS / Comparator: traditional   / Cmp polarity: active low  / Latching cmp: nonlatching / Cmp: disable
 * 1                   / 1XX   / 000           / 1                         / 100               / 0                         / 0                         / 0                         / 11
 * A0: 100; A1: 101; A2: 110; A3: 111
 * Config single shot A0 - 0xC183; A1 - 0xD183; A2 - 0xE183; A3 - 0xF183
 * Check convertion in bit 15 of config register (R0x01)
 * Read convertion from register R0x00
 * ADC adresses ADC0 - vo_1, vo_2, vo_3, vo_4 and ADC1 - vi 
 */
// Addresses of ADCs
static uint8_t ADC0_ADDR =  0x48; // Reads outputs of water level sensors
static uint8_t ADC1_ADDR =  0x49; // Reads Vcc
// ADC convertion and configuration registers
static uint8_t ADC_REG_CONV =  0x00; // Address of configuration register
static uint8_t ADC_REG_CONFIG =  0x01; // Address of configuration register
// ADC configuration words
static uint16_t A0_CONFIG = 0xC183; // Configuration words for all different analog inputs
static uint16_t A1_CONFIG = 0xD183;
static uint16_t A2_CONFIG = 0xE183;
static uint16_t A3_CONFIG = 0xF183;

// ---------- Define MACROS for serial communication protocol using binblock ----------
#define HEADER '#'
#define HEADER_REQUEST '$'

//  ---------- Define pump pins ----------
#define PUMP1_CTRL 9
#define PUMP2_CTRL 10
#define PUMP12_ENB 7

#define PUMP3_CTRL 5
#define PUMP4_CTRL 6
#define PUMP34_ENB 8

// Define timeout (ms)
#define TIMEOUT 2000

// ---------- Define structs to store data ----------
typedef struct quadrupleData {
    uint16_t v[5]; // output water level sensor (0-3) and input (4) 
    uint16_t u[2]; // input to 2 actuation pumps (10 bit each stored as 16 bit)
    uint8_t d[2]; // input to 2 disturbance pumps (8 bit each)
} quadrupleData;

// ---------- Read water level ----------
// This function reads the four output voltages and the input voltate to the struct quadruple
void readTankLevel();

// ---------- Control pumps ----------
void initPumps();
void pumpWrite();

// ---------- Broadcast water level data ----------
// This function broadcasts the water level information to simulink in a binblock 
// https://www.mathworks.com/help/instrument/binblockwrite.html
void levelDataSend();

// ---------- 16 bit resolution for pump inputs ----------
/* Configure digital pins 9 and 10 for 10-bit PWM output. */
void setupPWM10bit();
/* 10-bit version of analogWrite() for pins D9 and D10. */
void analogWrite10bit(uint8_t pin, uint16_t val);
