#include "sensors.h"
#include "main.h"
#include "debug.h"


//sensor maintenance.  Check for bad sensors. Regen if time, or giving bad readings.
void sensorMaint (){
  



  }


// Regenerate a T&H sensor
void regenSensorStart(uint8_t idx) {
  uint8_t address;

  if (idx == 0 || idx == 2) {
    address = SENSOR1_ADDR;
  } else {
    address = SENSOR2_ADDR;
  }

  if (idx < 2) {
    selectI2CChannel(I2C_CH_TEMPHUM_A);
  } else {
    selectI2CChannel(I2C_CH_TEMPHUM_B);
  }

  Wire.beginTransmission(address);
  Wire.write(0x2F);  // 110 mW for 1 s

  if (Wire.endTransmission() != 0) {
    DEBUG_PRINT(F("Regen Failed Sensor "));
    DEBUG_PRINTLN(idx);
    sensor[idx].regenState = FAILED;
  } else {
    sensor[idx].regenTimer = millis();
    sensor[idx].regenState = REGEN;
    sensor[idx].isValid = false;
    DEBUG_PRINT(F("Regen Sensor "));
    DEBUG_PRINTLN(idx);
  }
}

//Manage the starting and ending of T&H sensor regen cycles
void regenSensorEnd() {
  for (int i = 0; i < 4; i++) {                                    //scan SHT41As
    if (sensor[i].regenState == REGEN) {                           //do we have a sensor in regen?
      if (millis() - sensor[i].regenTimer >= REGEN_DURATION_MS) {  // 60 s heat + cool

        uint8_t address;  //find address of sensor
        if (i == 0 || i == 2) {
          address = SENSOR1_ADDR;
        } else {
          address = SENSOR2_ADDR;
        }

        if (i < 2) {  //find mux CH of sensor
          selectI2CChannel(I2C_CH_TEMPHUM_A);
        } else {
          selectI2CChannel(I2C_CH_TEMPHUM_B);
        }

        // Read and discard the heater measurement
        Wire.requestFrom(address, (uint8_t)6);
        while (Wire.available()) {
          Wire.read();
        }

        sensor[i].regenState = READY;              //set it back to ready
        sensor[i].isValid = true;                  //set it back to valid
        DEBUG_PRINT(F("Regen Complete Sensor "));  //let the world know
        DEBUG_PRINTLN(i);                          // that sensoo is back online
      }
    }
  }
}

// Selects one of the 8 channels on the TCA9548A multiplexer
void selectI2CChannel(uint8_t channel) {
  if (channel > 7) {
    DEBUG_PRINTLN(F("ERROR: Invalid mux channel (0-7 only)!"));
    return;
  }

  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
    DEBUG_PRINTLN(F("MUX select failed"));
    numberOfWireFaults++;          // or whatever counter you want
    // optional: try to recover, Wire.begin(), etc.
  }

  delayMicroseconds(10);
}

// read a T&H sensor
void readSHT41(uint8_t idx) {
  if (sensor[idx].regenState == REGEN || sensor[idx].regenState == FAILED) {
    return;  // don’t talk to it right now
  }
  uint8_t address;
  if (idx == 0 || idx == 2) {  //is this the 1st or 2nd sensor on the PCB
    address = SENSOR1_ADDR;
  } else {
    address = SENSOR2_ADDR;
  }
  if (idx < 2) {  //find which PCB and switch multiplexxer
    selectI2CChannel(I2C_CH_TEMPHUM_A);
  } else {
    selectI2CChannel(I2C_CH_TEMPHUM_B);
  }

  Wire.beginTransmission(address);
  Wire.write(0xFD);  // High precision measurement command for SHT4x
  if (Wire.endTransmission() != 0) {
    DEBUG_PRINT(F("T&H Sensor "));
    DEBUG_PRINT(idx);
    DEBUG_PRINTLN(F(" command failed"));
  } else {
    delay(10);  // SHT41 high-precision needs ~8.5 ms, 10 ms is safe
    Wire.requestFrom(address, (uint8_t)6);
    if (Wire.available() == 6) {
      uint16_t temp_raw = (Wire.read() << 8) | Wire.read();
      Wire.read();  // CRC (temperature)
      uint16_t humi_raw = (Wire.read() << 8) | Wire.read();
      Wire.read();  // CRC (humidity)

      // SHT4x conversion formulas
      float tempC = -45.0f + 175.0f * (temp_raw / 65535.0f);
      float tempF = tempC * 9.0f / 5.0f + 32.0f;

      float humid = -6.0f + 125.0f * (humi_raw / 65535.0f);
      if (humid > 100.0f) humid = 100.0f;
      if (humid < 0.0f) humid = 0.0f;

      CHAMBER_TEMP[idx].addValue(tempF);  //load values into running average
      CHAMBER_HUM[idx].addValue(humid);
      sensor[idx].temperature = CHAMBER_TEMP[idx].getAverage();
      sensor[idx].humidity = CHAMBER_HUM[idx].getAverage();
    }
  }
}

//sample all the sensors in the unit
void sampleSensors(void) {
  readSHT41(0);
  if (Wire.getWireTimeoutFlag()) {     //track wire resets **************
    Wire.clearWireTimeoutFlag();       //debug use, delete *************
    DEBUG_PRINTLN(F("T&H Sensor 0"));  //*********************
    numberOfWireFaults++;
  }
  readSHT41(1);
  if (Wire.getWireTimeoutFlag()) {     //track wire resets **************
    Wire.clearWireTimeoutFlag();       //debug use, delete *************
    DEBUG_PRINTLN(F("T&H Sensor 1"));  //*********************
    numberOfWireFaults++;
  }
  readSHT41(2);
  if (Wire.getWireTimeoutFlag()) {     //track wire resets **************
    Wire.clearWireTimeoutFlag();       //debug use, delete *************
    DEBUG_PRINTLN(F("T&H Sensor 2"));  //*********************
    numberOfWireFaults++;
  }
  readSHT41(3);
  if (Wire.getWireTimeoutFlag()) {     //track wire resets **************
    Wire.clearWireTimeoutFlag();       //debug use, delete *************
    DEBUG_PRINTLN(F("T&H Sensor 3"));  //*********************
    numberOfWireFaults++;
  }

  // === LM75B Ambient Temperature ===
  selectI2CChannel(I2C_CH_LOCAL_TEMP);
  Wire.beginTransmission(I2C_LOCAL_TEMP_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    Wire.requestFrom(I2C_LOCAL_TEMP_ADDRESS, 2);
    if (Wire.available() == 2) {
      int16_t raw = (Wire.read() << 8) | Wire.read();
      double ambF = ((raw >> 5) * 0.125) * 1.8 + 32.0;  // °F
      AMBIENT_TEMP.addValue(ambF);
    }
  }

  if (Wire.getWireTimeoutFlag()) {  //track wire resets **************
    Wire.clearWireTimeoutFlag();    //debug use, delete *************
    DEBUG_PRINTLN(F("LM75B"));      //*********************
    numberOfWireFaults++;
  }
  /*
  //selectI2CChannel(I2C_CH_TEMPHUM_A);
  //thanks GROK! I2C MUXs needs pullups on both sides.
  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(0x00);  // all channels off
  Wire.endTransmission();
  */
  // === NTC Sensors ===
  HEATBLOCK_TEMP.addValue(NTCread(NTC_HEATBLOCK_PIN));
  HEATSINK_TEMP.addValue(NTCread(NTC_HEATSINK_PIN));

  ambientTemp = AMBIENT_TEMP.getAverage();
  NTCtempHeatblock = HEATBLOCK_TEMP.getAverage();
  NTCtempHeatsink = HEATSINK_TEMP.getAverage();

  float sumTemp = 0;
  float sumHum = 0;
  uint8_t goodSensorCount = 0;

  for (int i = 0; i < 4; i++) {
    if (sensor[i].isValid) {
      sumTemp += sensor[i].temperature;
      sumHum += sensor[i].humidity;
      goodSensorCount++;
    }
  }

  if (goodSensorCount > 0) {
    temperature = sumTemp / goodSensorCount;
    humidity = sumHum / goodSensorCount;
    noSensorSince = 0;  // reset the timeout
    noSensorFault = false;
  } else {
    // keep the previous temperature & humidity values
    if (noSensorSince == 0) {
      noSensorSince = millis();  // start the timer the first time we have zero sensors
    }
    if (millis() - noSensorSince >= 120000UL) {  // e.g. 2 minutes
      noSensorFault = true;
      // optional: force safe PID output, raise alarm, etc.***********************
    }
  }
}

//read the NTCs
double NTCread(uint8_t pin) {
  double ADCvalue = analogRead(pin);
  double resistance = (1023.0 / ADCvalue) - 1.0;
  resistance = NTC_RESISTOR / resistance;
  double temp = resistance / NTC_RESISTANCE;
  temp = log(temp);
  temp /= NTC_B;
  temp += 1.0 / (NTC_REF + 273.15);
  temp = 1.0 / temp;
  return (temp - 273.15) * 1.8 + 32.0;
  //*pTemp -= 1.5f;
}
