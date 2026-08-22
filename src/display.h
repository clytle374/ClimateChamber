#pragma once

#include "main.h"   // so it can see the global variables and objects
#include <U8x8lib.h>          // if the functions need the display type

extern U8X8_SH1106_128X64_WINSTAR_4W_HW_SPI u8x8;

void displayUpdate(void);
void displayProgram(void);
int rightJustify(double value);
void digitalClockDisplay(uint8_t mode);
void printDigits(int digits);
