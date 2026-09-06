#pragma once

#include <Arduino.h>
#include <U8x8lib.h>  //https://github.com/olikraus/u8g2
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
//#include "PID_RT.h"
#include <QuickPID.h>       //https://github.com/Dlloydev/QuickPID
//#include <EncoderButton.h>  //https://github.com/Stutchbury/EncoderButton
#include <TimeLib.h>        //https://www.pjrc.com/teensy/td_libs_Time.html
//#include "climateChamber.h"
#include "RunningAverage.h"  ///https://github.com/RobTillaart/RunningAverage/tree/master
#include <avr/pgmspace.h>   //GROKS says so, works without 
#include <avr/wdt.h>        //watchdog 


// ***** CONSTANTS *****
// ***** GENERAL *****

#define DISPLAY_RATE  5000UL   //display update normal
#define DISPLAY_PROGRAM_RATE  125UL //display update for menus
#define CONTROL_RATE  5000UL //general control loop 
#define PID_RATE  5000000UL  //PID control ***in uS NOT mS 
#define SENSOR_RATE  1000UL  // time for reads   
#define TEMP_FAULT_THRESHOLD 5      // 5F diffrence is a dead sensor. 
#define HUMIDITY_FAULT_THRESHOLD 5  //5 percent diff is a sensor that needs regen
#define SCREEN_LINES_HIGH 8     //home many lines can we put on the screen  
#define SCREEN_LINES_WIDTH 16  //how many charters can we fit across the screen 
#define STATUS_SCREEN_ENTRIES 15 //how many enteries do we have in the status screen?


// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0    //eeprom address for current profile
#define EEPROM_STORAGE_ADDRESS 1  //eeprom address for everything else
#define DEAD_ZONE 1               //degrees of dead zone
#define DEAD_ZONEp 0              //^^ parameter number
#define INCUBATE_TEMP 93          //temp for
#define INCUBATE_TEMPp 1          //^^ parameter number
#define INCUBATE_HUMIDITY_SET 75  //huisity for
#define INCUBATE_HUMIDITY_SETp 2  //^^ parameter number
#define FRUIT_TEMP 63             //temp for
#define FRUIT_TEMPp 3             //^^ parameter number
#define FRUIT_HUMIDITY_SET 90     //humisity for
#define FRUIT_HUMIDITY_SETp 4     //^^ parameter number
#define LIGHT_ON_HOURS 12         //light per day
#define LIGHT_ON_HOURSp 5         ////^^ parameter number
#define LIGHT_LEVEL 7         //0-7 light brightness
#define LIGHT_LEVELp 6         //0-7 light brightness
#define WATER_SECONDS 3     //fire ultrasonic time seconds
#define WATER_SECONDSp 7     //^^ parameter number
#define AIR_SECONDS 95     //seconds for the air pump to run
#define AIR_SECONDSp 8    //^^ parameter number
#define PID_KP_HEAT 26
#define PID_KP_HEATp 9            //^^ parameter number
#define PID_KI_HEAT 0.022
#define PID_KI_HEATp 10           //^^ parameter number
#define PID_KD_HEAT 0.011
#define PID_KD_HEATp 11           //^^ parameter number
#define PID_P_BLOCK_HEAT  15
#define PID_P_BLOCK_HEATp 12       //^^ parameter number
#define PID_KP_COOL 26
#define PID_KP_COOLp 13           //^^ parameter number
#define PID_KI_COOL 0.022
#define PID_KI_COOLp 14           //^^ parameter number
#define PID_KD_COOL 0.011
#define PID_KD_COOLp 15           //^^ parameter number
#define PID_P_BLOCK_COOL 24
#define PID_P_BLOCK_COOLp 16       //^^ parameter number
#define STATUS_SECONDS 35    //how long to display status screen 
#define STATUS_SECONDSp 17  //^^ parameter number
#define SET_HOURS 1               //set time hours
#define SET_HOURSp 18             //^^ parameter number
#define SET_MINUTES 1             //set time minutes
#define SET_MINUTESp 19           //^^ parameter number
#define SET_MONTH 9               //set time minutes
#define SET_MONTHp 20             //^^ parameter number
#define SET_DAY 17             //set time minutes
#define SET_DAYp 21                  //^^ parameter number
#define SET_YEAR 26                 //set time minutes
#define SET_YEARp 22                //^^ parameter number
#define SERIAL_SPEED 115200  //serial com speed parameter 36
#define SERIAL_BAUD 23       //^^ parameter number
//********** end paramters saved in eeprom *****************
//pointers for specail parameters for menus
#define EXIT 24
#define DUMP_TO_SERIAL 25
#define LOAD_DEFAULTS 26
#define SAVE_TO_EEPROM 27

//Pin assignments
#define ROTARY_PIN1 10  //PD2
#define ROTARY_PIN2 11  //PD3
#define BUTTON_PIN 12   //PD4
#define LIGHT_PIN1 18  //dac light pins 
#define LIGHT_PIN2 19  //dac light pins 
#define LIGHT_PIN3 20  //dac light pins 
#define LIGHT_PIN4 21  //dac light pins
#define HUMIDIFIER_PIN 28 // ultrasonic humidifier
#define PELTIER_ON_PIN 15 // Peltier on 
#define PELTIER_REV_PIN 14 // Peltier reverse polarity 
#define AIRPUMP_PIN 27 // airpump 
//****************** SWAPPED DUE CTO BAD PIN*********************** */
#define NTC_HEATSINK_PIN 25// NTC
#define NTC_HEATBLOCK_PIN 24 // NTC 
#define NTC_RESISTOR 10000
#define NTC_RESISTANCE 10000
#define NTC_REF 25
#define NTC_B 3950
#define RUN_LED 22
#define HEAT_LED 23
#define PWR_LED 31
#define AIR_LED 30
#define H2O_LED 29
#define ERROR_LED 28
#define FAN_PWM 13

//mux addresses 
#define I2C_CH_TEMPHUM_A    0   //sensor board 1
#define I2C_CH_TEMPHUM_B    1   //sensor board 2
#define I2C_CH_LOCAL_TEMP   4
#define I2C_LOCAL_TEMP_ADDRESS 0x48  //LM75B sensor address 
#define I2C_CH_RTC          5
#define I2C_CH_SPARE1       2
#define I2C_CH_SPARE2       3

//I2C device addresses 
#define I2C_MUX_ADDR        0x77
#define SENSOR1_ADDR 0x44           //temp and humidity I2C addresses
#define SENSOR2_ADDR 0x45           //second temp and humidity I2C sensor
#define REGEN_DURATION_MS 60000UL   // 60 seconds heat + cool-down
#define I2C_LOCAL_TEMP_ADDRESS 0x48  //LM75B sensor address

//OLED SW SPI setup     ****I'm and idiot, use MISO for the data to the OLED on the PCB
#define OLED_CS 7  // Example Clock pin (SCK)
#define OLED_DC  4  //  DC
#define OLED_RESET 2  // Example Reset pin (RES)


// ===== Global variables =====
extern float temperature;               // I2C temp calculated
extern float humidity;                  // I2C humidity calculated
extern float NTCtempHeatblock;          // NTC reading
extern float NTCtempHeatsink;           // NTC reading
extern float humiditySetpoint;          // target humidity (moved for status display)
extern float ambientTemp;               // MB ambient temp
extern double px[41];                   // parameters
extern int encoder;                     // stored encoder counts
extern bool button;                     // is button pressed?
extern int currentParam;                // current position of the programming menu
extern int lastPointer;                 // save state to catch change
extern int menuPointer;                 // selector for parameters
extern bool edit;                       // edit mode flag
extern int menuYes;                     // triggers YES/NO display
extern int currentBaudPointer;          // baudrate list pointer

// control timer variables
extern unsigned long controlTimer;
extern unsigned long sensorTimer;
extern unsigned long statusScreenTimer;
extern unsigned long humidifierRunningTimer;
extern unsigned long airPumpRunningTimer;
extern unsigned long previousMillis;
extern unsigned long dynamicInterval;
extern float heatblockDeltaT;
extern int lastMinute;
extern int lastHour;
extern int last6thHour;

extern int setYear;
extern int setMonth;
extern int setDay;
extern int setHours;
extern int setMinutes;

extern bool airPumpRunning;
extern bool humidifierRunning;
extern uint8_t fanSpeed;
extern uint8_t fanTarget;
extern bool inHeatMode;
extern bool runProgramCode;
extern bool runMenuCode;
extern bool runStatusCode;
extern bool StatusModeStarted;
extern int menuSelector;
//extern bool humidifierRan;
extern bool useOuterI;

// screen scrolling
extern int screenX;
extern int screenY;
extern int xShift;

// PID control variables
extern float setpoint;
extern float pwmDrive;
extern float heatBlockInput;
extern float PID1output;
extern unsigned long noSensorSince;
extern bool noSensorFault;
extern uint16_t numberOfWireFaults;

extern float heatblockLastTemp;
extern bool ntcBlockRawOk;
extern bool ntcSinkRawOk;
extern bool systemIsHalted;

extern RunningAverage AMBIENT_TEMP;
extern RunningAverage HEATBLOCK_TEMP;
extern RunningAverage HEATSINK_TEMP;
extern RunningAverage PWM_DRIVE;
//these are for the 4 SHT41A sensors
extern RunningAverage CHAMBER_TEMP[4]; 
extern RunningAverage CHAMBER_HUM[4]; 

// states for display modes
typedef enum DISPLAY_MODE {
  RUN,
  MENU,
  STATUS,
  PROGRAM
} displayMode_t;
extern displayMode_t displayMode;

// overall function modes
typedef enum MODE_STATE {
  INCUBATE,
  FRUIT
} modeState_t;
extern modeState_t modeState;

enum RegenState {
  READY,
  REGEN,
  FAILED,
  REHAB
};

struct SensorState {
  float temperature;
  float humidity;
  bool isValid;
  RegenState regenState;
  unsigned long regenTimer;
  unsigned long regenDuration;
  uint8_t lastRegenDay;
  uint16_t faultCounter;
};
extern SensorState sensor[4];

extern const char menuOptions[][4];
extern const char* const monthNames[] PROGMEM;
extern const char* const parameterNames[] PROGMEM;
extern const float defaultpx[30] PROGMEM;
extern const float baudRates[14];



//extern EncoderButton eb1;
extern QuickPID chPID;
extern QuickPID hbPID;

extern volatile int8_t encDelta;

// ========== Function prototypes (temporary – everything in one place for now) ==========
void detectHCmode();
void updateUseOuterI();
void loadPIDs(void);
void swapPIDmode(void);
void setFanSpeed();

void minutesFunctions(unsigned long now);
void hourFunctions(unsigned long now);
void lightControl(bool state);

void checkForMosfetFailure(void);
void confirmMosfetShort(void);
void scuttleShip(void);

