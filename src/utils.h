#pragma once

#include "main.h"   // so it can see the global variables and objects
#include "utils.h"

int8_t encoderRead(void);
void EBupdate(void);
void encoderSetup(void);
void encStep(void);
//void onEb1Clicked(EncoderButton& eb);
//void onEb1Encoder(EncoderButton& eb);
void programmingMenu(void);
void menuMenu(void);
double constrainValue(uint8_t param, double val);
uint8_t maxDayInMonth(uint8_t month, uint16_t year);
uint8_t decToBcd(uint8_t val);
uint8_t bcdToDec(uint8_t val);
time_t getRtcTime();
void updateRTC();
void writeFullRtcFromSysTime();
int freeRam();