/**************************************************************************
  This is a library for Nokia 1.8" displays based on SPFD54124B drivers.
  Works with the Custom 1.8" Display Breakout board
  Check out the Github of Library for our tutorials and wiring diagrams.
  These displays use 9-bit SPI to communicate, 3 pins are required to
  interface.
  Abhishek Tiwari invests time and resources providing this open source code,
  please support Abhishek Tiwari and open-source hardware by Providing some Credits or mentions.
  Written by Abhishek Tiwari.

  This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 **************************************************************************/


/* NOKIA TFT 105,C1-00,1616 DISPLAY 128x166, 65K COLORS, 1.8 INCH, CODE: ET-LCD-104176
  12 PIN
//-----------------------
  | 1:  GND             |
  | 2:  RESET           |
  | 3:  CS              |
  | 4:  GND             |
  | 5:  SDA -> MOSI     |
  | 6:  SCK             |
  | 7:  VDDI            |
  | 8:  VDD             |
  | 9:  GND             |
  | 10: LED-            |
  | 11: LED+            |
  | 12: GND             |
//-----------------------
*/
//Currently GFX library is not for AVR Microcontroller

#ifdef ARDUINO_ARCH_AVR
const int _CS         = 10;
const int _RESET      = 12;
const int SCLK        = 13;
const int SID         = 11; //Mosi

#elif defined ARDUINO_ARCH_ESP32
const int _CS         = 15;
const int _RESET      = 19;
const int SCLK        = 18;
const int SID         = 23; //Mosi
#include <Adafruit_GFX.h>
#elif defined ARDUINO_ARCH_ESP8266 
const int _CS         = 15;          
const int _RESET      = 16;       
const int SCLK        = 14;  
const int SID         = 13; //Mosi
#include <Adafruit_GFX.h>
#else 
    #error This library only supports boards with an AVR, ESP32, and ESP8266 processor.
#endif  

#include <lcd.h>
#include "ben10.h"

#define BAUD_RATE     9600
//Arduino uno: PWM Brightness pin 9: PB1: 15: OC1A
//ESP32: PWM Brightness pin 4
Nokia105 display( SID,  SCLK, _RESET, _CS);

void setup() {
if (LOG) {
  Serial.begin(BAUD_RATE);
  Serial.println("Display Debug");
}

display.initDisplay();
display.PWMinit();
display.setLcdBrightness(1000);  
display.setDrawPosition(128,160); 
display.backgroundColor(WHITE);
display.displayClear();
display.setLcdBrightness(1000); //16 BIT value only
}

void loop() {
display.drawRGBBitmap(4, 10, imageaccelarate, 120,159);
display.drawRGBBitmap(4, 10, imagediamondhead, 120,159);
display.drawRGBBitmap(4, 10, imagefourarms, 120,159);
display.drawRGBBitmap(4, 10, imageghostfreak, 120,159);
display.drawRGBBitmap(4, 10, imagegreymatter, 120,159);
display.drawRGBBitmap(4, 10, imageheatblast, 120,159);
display.drawRGBBitmap(4, 10, imagestinkfly, 120,159);
display.drawRGBBitmap(4, 10, imageupgrade, 120,159);
display.drawRGBBitmap(4, 10, imagewildboar, 120,159);
display.drawRGBBitmap(4, 10, imageripjaw, 120,159);
}
