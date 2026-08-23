#pragma once

#include "main.h"   // so it can see the global variables and objects

void sensorMaint();
void sensorTempTest();
void sensorHumidityTest();
void regenSensorStart(uint8_t idx);
bool regenSensorEnd(uint8_t idx);
void selectI2CChannel(uint8_t channel);
void readSHT41(uint8_t idx);
void sampleSensors(void);
double NTCread(uint8_t pin);