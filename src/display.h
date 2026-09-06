#pragma once

#include "main.h"   // so it can see the global variables and objects
#include <U8x8lib.h>          // if the functions need the display type

extern U8X8_SH1106_128X64_WINSTAR_4W_HW_SPI u8x8;
// Create the u8x8 display                //
// U8X8_SH1106_128X64_WINSTAR_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC,
// /* reset=*/OLED_RESET);  // same as the NONAME variant, but uses updated
// SH1106 init sequence

// *****************one of these shoulf work for new display
// *************************
// U8X8_SSD1327_EA_W128128_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /*
// reset=*/OLED_RESET);   //try this U8X8_SSD1327_WS_128X128_4W_HW_SPI u8x8(/*
// cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);   /or this if text is
// shifted u8x8.setContrast(value);   //screen dimming, it doesn't belong here,
// it accepts 0-255
// u8x8.sendF("ca", 0x81, 0x00);  //deeper dimming last input accepts 0x00 to
// 0x0F
void displayUpdate(void);
void displayProgram(void);
int rightJustify(double value);
void digitalClockDisplay(uint8_t mode);
void printDigits(int digits);
