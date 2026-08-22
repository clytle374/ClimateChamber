/* climate chamber controller.  Using DF robot temp and humidity sensor. cheap 128x64 6 pin spi u8x8 display.
NTC.  Cheap ultrasonic huidifier element.  Some white LEDs for lighting
Started out with one of those pelita junction 6 can refrigators.  
It's designed to grow small batches of delicious mushrooms, use it for whatever.
Code has been snipped, rewritten, modded, rewritten and modified again
from examples across the internet, and many of them recycled beyond tracing 
on several of my project.  Libaries have licenses.  GROK has both helped amd screwed me.
Use it for whatever 

************************************** TOO DO LIST *********************************

*/

#include "main.h"
#include "display.h"
#include "sensors.h"
#include "debug.h"

float temperature;  //I2C temp calculated
float humidity;     //I2C huidity calculated
float NTCtempHeatblock;   //where we put NTC reading
float NTCtempHeatsink;    //where we put NTC reading
float humiditySetpoint;   //target temp, moved here due to status display
float ambientTemp = 100;  //MB ambient temp  I hope they are close
double px[41];  //parameters
int encoder;                 //stored encoder couunts
bool button = 0;             // is button pressed?
int currentParam = 0;        //what is the current position of the programming menu
int lastPointer;             //save state to catch change
int menuPointer = 10;        // move selector to parameters.  10 is for changing selected parameter currentParam 0-9 pick a digit
bool edit = 1;               // sw back to edit mode
int menuYes = 0;             // IDK, look into this  I thinkg this triggers the YES NO display in program memory
int currentBaudPointer = 0;  // pointer for selecting baudrates from list, should be local?
//control timer variables
unsigned long controlTimer;              //control internal time
unsigned long sensorTimer;               // time for reads
unsigned long statusScreenTimer;         //pid loop time
unsigned long humidifierRunningTimer;    //when did we start the humidifier
unsigned long airPumpRunningTimer;       //when did we start the airpump
unsigned long previousMillis = 0;        // Timestamp of the last execution
unsigned long dynamicInterval = 1000UL;  // The current interval duration
bool minutesFunctionsRan = 0;            // only run the minutes function once per true test
int lastMinute = 0;                      // keep track of the last run on minutes scheduler
bool hourFunctionsRan = 0;               // did we do hour functions this hour?
int lastHour = 0;                        // keep track of the last hour function run
int setYear;                             // year for setting parameters
int setMonth;                            // month for setting parameters
int setDay;                              // day for setting parameters
int setHours;                            // hours for setting parameters
int setMinutes;                          // minutes for setting parameters.
bool airPumpRunning = false;             //airpump
bool humidifierRunning = false;
uint8_t fanSpeed = 100;      // current actual speed (global or static)
uint8_t fanTarget = 25;      // Desired fan speed (updated from main control logic)
bool inHeatMode = true;      //PID to setup for heat
bool runProgramCode = 0;     //switch on the programming code
bool runMenuCode = 0;        //switch on the menufunctions code
bool StatusModeStarted = 0;  //are we going to be in status code?
int menuSelector = 3;        //selecting menu options
bool humidifierRan = 0;      //just for serial output as humidifer runs for a time shorter than update.
bool useOuterI = true;       // are we inhibiting the PID I term to stop the endless windup problem
//screen scrolling variables.
int screenX = 0;  //locations for display scanning to not burn screen
int screenY = 0;  //locations for display scanning to not burn screen
int xShift;       //used to return a variable from a right justify function
// ***** PID CONTROL VARIABLES *****
float setpoint;  //target temp
//double input = 21;    //zero will throw errors before first averaged sample
float pwmDrive;        //output from PID for main element
float heatBlockInput;  //output for the cool PID
float PID1output;      //we're going to save the output of PID1 here and use it for deltaT and serial debugging
unsigned long noSensorSince = 0;  //timer to start if e have no sensor data
bool noSensorFault = false;       //fault state for no sensor data.
uint16_t numberOfWireFaults = 0;  //logging the number of wire timeouts, crash hunting

// setup running averages
RunningAverage AMBIENT_TEMP(5);
RunningAverage HEATBLOCK_TEMP(5);
RunningAverage HEATSINK_TEMP(5);
//these are for the 4 SHT41A sensors
RunningAverage CHAMBER_TEMP[4] = {
  RunningAverage(5),
  RunningAverage(5),
  RunningAverage(5),
  RunningAverage(5)
};
RunningAverage CHAMBER_HUM[4] = {
  RunningAverage(5),
  RunningAverage(5),
  RunningAverage(5),
  RunningAverage(5)
};
SensorState sensor[4];  // sensor[0] through sensor[4]

displayMode_t displayMode;
modeState_t modeState;


// these are the texts for the menu.
const char menuOptions[][4] = { "NO", "YES" };  //display for ON/OFF menu options
//add the names of the 12 months to output to the OLED
const char* const monthNames[] PROGMEM = { "Jan",
                                           "Feb",
                                           "Mar",
                                           "Apr",
                                           "May",
                                           "Jun",
                                           "Jul",
                                           "Aug",
                                           "Sep",
                                           "Oct",
                                           "Nov",
                                           "Dec" };
//name all the paramaters for the programming menu
const char* const parameterNames[] PROGMEM = { "DEAD_ZONE",

                                               "INCUBATE_TEMP",
                                               "INCUBATE_HUMID",
                                               "FRUIT_TEMP",
                                               "FRUIT_HUMID",
                                               "LIGHT_ON_HOURS",
                                               "LIGHT_LEVEL",
                                               "WATER_SECONDS",
                                               "AIR_SECONDS",
                                               "CH_P_HEAT",
                                               "CH_I_HEAT",
                                               "CH_D_HEAT",
                                               "BL_P_HEAT",
                                               "CH_P_COOL",
                                               "CH_I_COOL",
                                               "CH_D_COOL",
                                               "BL_P_COOL",
                                               "STATUS_SEC",
                                               "SET_HOUR",
                                               "SET_MINUTE",
                                               "SET_MONTH",
                                               "SET_DAY",
                                               "SET_YEAR",
                                               "SERIAL_SPEED",
                                               "EXIT",
                                               "DUMP_TO_SERIAL",
                                               "LOAD_DEFAULTS",
                                               "SAVE" };



//these are the defaults for the parameters. I think?  GD am a ture
const float defaultpx[30] PROGMEM = { DEAD_ZONE,
                                      INCUBATE_TEMP,
                                      INCUBATE_HUMIDITY_SET,
                                      FRUIT_TEMP,
                                      FRUIT_HUMIDITY_SET,
                                      LIGHT_ON_HOURS,
                                      LIGHT_LEVEL,
                                      WATER_SECONDS,
                                      AIR_SECONDS,
                                      PID_KP_HEAT,
                                      PID_KI_HEAT,
                                      PID_KD_HEAT,
                                      PID_P_BLOCK_HEAT,
                                      PID_KP_COOL,
                                      PID_KI_COOL,
                                      PID_KD_COOL,
                                      PID_P_BLOCK_COOL,
                                      STATUS_SECONDS,
                                      SET_HOURS,
                                      SET_MINUTES,
                                      SET_MONTH,
                                      SET_DAY,
                                      SET_YEAR,
                                      SERIAL_SPEED,
                                      0,
                                      0,
                                      0,
                                      0 };

//availble baud rates, did I miss anything important?
const float baudRates[14] = { 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600,
                              74800, 115200, 230400, 250000, 500000, 1000000 };


//***********************stopped switching move to header file here */
                              // Create the u8x8 display                //
//U8X8_SH1106_128X64_WINSTAR_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);  // same as the NONAME variant, but uses updated SH1106 init sequence

// *****************one of these shoulf work for new display *************************
//U8X8_SSD1327_EA_W128128_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);   //try this
//U8X8_SSD1327_WS_128X128_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);   /or this if text is shifted
//u8x8.setContrast(value);   //screen dimming, it doesn't belong here, it accepts 0-255
// u8x8.sendF("ca", 0x81, 0x00);  //deeper dimming last input accepts 0x00 to 0x0F

EncoderButton eb1(ROTARY_PIN1, ROTARY_PIN2, BUTTON_PIN);  //sets up the encoder buton
// setup the 2 PID functions, in cascade
//Heat/cool and paramters are switched when switching HEAT/COOL function
QuickPID chPID(&temperature, &PID1output, &setpoint);           //outer loop
QuickPID hbPID(&NTCtempHeatblock, &pwmDrive, &heatBlockInput);  //inner loop



void setup() {
  //modeState = INCUBATE;  //start here for testing *********************TESTING****************
  displayMode = RUN;                   // put display in run to start out      *************************
  pinMode(ROTARY_PIN1, INPUT_PULLUP);  //************************added are they needed?*******************
  pinMode(ROTARY_PIN2, INPUT_PULLUP);  //************************added are they needed?*******************
  pinMode(BUTTON_PIN, INPUT_PULLUP);   //************************added are they needed?*******************
  pinMode(LIGHT_PIN1, OUTPUT);
  pinMode(LIGHT_PIN2, OUTPUT);
  pinMode(LIGHT_PIN3, OUTPUT);
  pinMode(LIGHT_PIN4, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(PELTIER_ON_PIN, OUTPUT);
  pinMode(PELTIER_REV_PIN, OUTPUT);
  pinMode(AIRPUMP_PIN, OUTPUT);
  pinMode(NTC_HEATBLOCK_PIN, INPUT);
  pinMode(NTC_HEATSINK_PIN, INPUT);
  pinMode(RUN_LED, OUTPUT);
  pinMode(HEAT_LED, OUTPUT);
  pinMode(PWR_LED, OUTPUT);
  pinMode(AIR_LED, OUTPUT);
  pinMode(H2O_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);

  // === Peltier PWM on PD7 (OC2A) ===
  pinMode(PELTIER_ON_PIN, OUTPUT);  // PD7

  // Timer 2: Fast PWM, non-inverting
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS20);  // Prescaler 1 → ~78 kHz
  OCR2A = 255;           // Start at 0%

  // === Fan PWMs ===
  pinMode(FAN_PWM, OUTPUT);               // PD5 as output (OC1A)
                                          // Timer 1: 8-bit Fast PWM, non-inverting on OC1A
  TCCR1A = (1 << COM1A1) | (1 << WGM10);  // WGM11=0, WGM10=1  → 8-bit mode
  TCCR1B = (1 << WGM12) | (1 << CS11);    // WGM12=1, prescaler 8

  OCR1A = 255;  // Start at 100%




  for (int i = 0; i < EXIT; i++) {  //this loop loads eeprom into parameters
    //EEPROM.put(EEPROM_STORAGE_ADDRESS + (i*4) , px[i]);   //this loads eeprom from parameters use this for new baord  ******doesn't work******
    EEPROM.get(EEPROM_STORAGE_ADDRESS + (i * 4), px[i]);  //this loads parameters from eeprom
  }
  for (int i = EXIT; i < SAVE_TO_EEPROM; i++) {  //clear the commands parameters
    px[i] = 0;
  }

  unsigned char value;                      // = EEPROM.read(PROFILE_TYPE_ADDRESS);
  EEPROM.get(PROFILE_TYPE_ADDRESS, value);  // resume last state in case of power fail
  if ((value == 0) || (value == 1)) {
    // Valid mode?

    if (value == 1) {
      modeState = FRUIT;
      setpoint = px[FRUIT_TEMPp];
      humiditySetpoint = px[FRUIT_HUMIDITY_SETp];
    } else {
      modeState = INCUBATE;
      setpoint = px[INCUBATE_TEMPp];
      humiditySetpoint = px[INCUBATE_HUMIDITY_SETp];
    }
  } else {
    // Default to fruit if eeprom invalid
    EEPROM.put(PROFILE_TYPE_ADDRESS, 0);
    modeState = FRUIT;
    setpoint = px[FRUIT_TEMPp];
    humiditySetpoint = px[FRUIT_HUMIDITY_SETp];
  }


  // Initialize sensors
  for (uint8_t i = 0; i < 4; i++) {
    sensor[i].temperature = 0;
    sensor[i].humidity = 0;
    sensor[i].isValid = true;
    sensor[i].regenState = READY;
    sensor[i].regenTimer = 0;
    sensor[i].lastRegenDay = 0;
  }




  //DEBUG_BEGIN(115200);  //set serial speed
  DEBUG_BEGIN(px[SERIAL_BAUD]);  //set serial speed
  DEBUG_PRINTLN("hello");

  //encoder switch events
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  Wire.begin();
  Wire.setWireTimeout(25000, true);  // 25 ms timeout, reset TWI hardware on timeout
  u8x8.begin();                      //display startup
                                     //u8x8.setPowerSave(0);
  u8x8.setFlipMode(0);               // Flip the display 180 degrees

  //clear running averages
  for (int i = 0; i < 4; i++) {
    CHAMBER_TEMP[i].clear();
    CHAMBER_HUM[i].clear();
  }
  AMBIENT_TEMP.clear();
  HEATBLOCK_TEMP.clear();
  HEATSINK_TEMP.clear();


  sampleSensors();

  if (temperature > setpoint) {
    inHeatMode = false;
  }


  chPID.SetOutputLimits(-100, 100);
  hbPID.SetOutputLimits(0, 100);
  chPID.SetSampleTimeUs(PID_RATE);
  hbPID.SetSampleTimeUs(PID_RATE);
  chPID.SetMode(chPID.Control::automatic);
  hbPID.SetMode(hbPID.Control::automatic);
  chPID.SetAntiWindupMode(chPID.iAwMode::iAwClamp);  //anti-windup control
  hbPID.SetAntiWindupMode(hbPID.iAwMode::iAwClamp);  //anti-windup control

  updateUseOuterI();  // Set the initial state of useOuterI based on current error
  swapPIDmode();      // set the
  loadPIDs();         //loads the P,I,D values into the PIDs

  selectI2CChannel(I2C_CH_RTC);
  setSyncProvider(getRtcTime);
  lightControl(false);
}


void loop() {                              //******************main loop************************
  unsigned long currentMillis = millis();  // Get the current time
  eb1.update();                            //button update
  // Ask if outer PID loop is ready to execute
  if (chPID.Compute()) {                                    //if PID1 runs, its timebase is controlled internally. Lets use the output to make a useable input for PID2
    float deltaT = PID1output * 0.33f;                      //lets scale PID1's output to 0 to X degrees F over the current chamber temp
    float ambientOffset = (setpoint - ambientTemp) * 0.1f;  //feed forward the ambient vs setpoint in increase control range on heatblock
    heatBlockInput = temperature + deltaT + ambientOffset;  // Aim for delta from process
  }
  // Ask if inner PID loop is ready to execute
  if (hbPID.Compute()) {  //call PID2, if it returns that it ran, spit out debugging info

    //*********************


    // Convert to 8-bit value (0-255)
    uint8_t dutyCycle = (uint8_t)(pwmDrive * 2.55 + 0.5);  // +0.5 for better rounding

    // Inverted for your hardware (100% PWM = OCR2A = 0)
    SET_PWM(dutyCycle);  //OCR2A = 255 - dutyCycle;

    DEBUG_PRINT(humidity);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(inHeatMode);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(temperature);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(setpoint);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(PID1output);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(NTCtempHeatblock);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(heatBlockInput);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(pwmDrive, 0);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(NTCtempHeatsink);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(humidifierRan);
    humidifierRan = 0;
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(airPumpRunning);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(fanSpeed);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(ambientTemp);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(useOuterI);
    DEBUG_PRINT(F(","));
    DEBUG_PRINTLN(numberOfWireFaults);
    DEBUG_FLUSH();  // Keeps the CPU here until the talk is done
  }
  //sample sensors every 1000 millis
  if (currentMillis - sensorTimer >= SENSOR_RATE) {

    sensorTimer = currentMillis;  // this is rolloever safe acording to AI
    sampleSensors();
    digitalWrite(RUN_LED, !digitalRead(RUN_LED));
    setFanSpeed();
  }
  //test if it's time to update the display
  if (currentMillis - previousMillis >= dynamicInterval) {  //update DISPLAY
    previousMillis = currentMillis;                         // Get current time once
    displayUpdate();
  }
  //test if it's time to timeout the status display.  Don't burn the OLED display
  if (StatusModeStarted == true && currentMillis - statusScreenTimer >= (px[STATUS_SECONDSp] * 1000)) {  //*****lets setup a detailed status page display, we'll use this to time it out to not burn the screem****************
    displayMode = RUN;
    StatusModeStarted = false;
  }
  //run the general control loop every 5 seconds, 5000 miillis
  if (currentMillis - controlTimer >= CONTROL_RATE) {  //run controls   5000
    controlTimer = currentMillis;                      // this is rolloever safe acording to AI
    //raise and lower the sun.
    double minutesNow = (hour() * 60) + minute();
    double lightMinutes = px[LIGHT_ON_HOURSp] * 30;                                                    //  number of minutes before and after noon light should be on hours * 60 / 2
    if ((minutesNow > 720 - lightMinutes && minutesNow < 720 + lightMinutes) && modeState == FRUIT) {  //are we in fruit and is it time for lights? ************FIX THIS, LIGHT_TIME IGNORED**********
      lightControl(true);
    } else {
      lightControl(false);
    }
    //lets do some stuff ever 5 MINUTES  **********change back to 5 minutes****************
    if (minutesFunctionsRan == 0 && (minute() % 2 == 0) && minute() != lastMinute) {
      lastMinute = minute();
      minutesFunctions(currentMillis);
      minutesFunctionsRan = 1;
      //DEBUG_PRINTLN(F("minute functions ran"));
    } else {
      minutesFunctionsRan = 0;
    }
    //lets do some stuff ever hour
    if (hourFunctionsRan == 0 && hour() != lastHour) {
      lastHour = hour();
      hourFunctions(currentMillis);
      hourFunctionsRan = 1;
      //DEBUG_PRINTLN(F("hour functions ran"));
    } else {
      hourFunctionsRan = 0;
    }
    detectHCmode();
    updateUseOuterI();
    //setFanSpeed(40);
  }
  if (runMenuCode == true) {  //code for changeing function withing the menu
    menuMenu();
  }
  if (runProgramCode == true) {  //are we in programing mode?
    programmingMenu();
  }
  if (humidifierRunning && (currentMillis - humidifierRunningTimer >= (px[WATER_SECONDSp] * 1000))) {  //shutdown humidifier when time is up
    humidifierRunning = false;
  }
  if (airPumpRunning && (currentMillis - airPumpRunningTimer >= (px[AIR_SECONDSp] * 1000))) {  //shutdown the airpump when time is up
    airPumpRunning = false;
  }

  //  WRITE TO THE OUTPUTS NOw THAT WE'RE AT THE END
  digitalWrite(HUMIDIFIER_PIN, humidifierRunning);
  digitalWrite(AIRPUMP_PIN, airPumpRunning);
  // **********************  status LEDS *********************
  digitalWrite(H2O_LED, humidifierRunning);
  digitalWrite(AIR_LED, airPumpRunning);
  regenSensorEnd();  //check and see if a sensor is done with regeneration
}

// detect if we need to swap between heat and cool mode
void detectHCmode() {

  if (PID1output > 0) {
    // Temperature is too low, definitively needs heat
    if (inHeatMode == false) {  // Only print/switch if changing state
      DEBUG_PRINTLN(F("Force to Heat Mode"));
      inHeatMode = true;
      swapPIDmode();
    }
  }
  if (PID1output < 0) {
    // Temperature is too high, definitively needs cooling
    if (inHeatMode == true) {  // Only print/switch if changing state
      DEBUG_PRINTLN(F("Force to Cool Mode"));
      inHeatMode = false;
      swapPIDmode();
    }
  }
}

//detect if we are withing half of deadzone from target to change loop agressiveness
void updateUseOuterI() {
  static bool lastUseOuterI = false;

  float error = abs(setpoint - temperature);

  // Turn I ON when error gets small enough (with some hysteresis)
  if (!useOuterI && error < px[DEAD_ZONEp] * 2) {
    useOuterI = true;
  }

  // Turn I OFF only when error gets clearly larger
  if (useOuterI && error > px[DEAD_ZONEp] * 2) {
    useOuterI = false;
  }

  // Only update PID gains if the state actually changed
  if (useOuterI != lastUseOuterI) {

    loadPIDs();

    lastUseOuterI = useOuterI;
  }
}

// Load the correct P, I, D values into both PIDs
void loadPIDs(void) {
  float kp, ki, kd, blockP;

  if (inHeatMode) {
    kp = useOuterI ? px[PID_KP_HEATp] : px[PID_KP_HEATp] * 1.0f;  //was 1.5, testing no boost
    ki = useOuterI ? px[PID_KI_HEATp] : 0.0f;
    kd = px[PID_KD_HEATp];
    blockP = px[PID_P_BLOCK_HEATp];
  } else {
    kp = useOuterI ? px[PID_KP_COOLp] : px[PID_KP_COOLp] * 1.0f;  //was 1.5, testing no boost
    ki = useOuterI ? px[PID_KI_COOLp] : 0.0f;
    kd = px[PID_KD_COOLp];
    blockP = px[PID_P_BLOCK_COOLp];
  }

  chPID.SetTunings(kp, ki, kd);


  hbPID.SetTunings(blockP, 0, 0);  // Inner loop stays simple
}

// handle swapping the inner loop direction and safely switching the relay
void swapPIDmode(void) {
  // 1. Kill the output hard (important with inverted PWM)
  OCR2A = 255;
  delay(100);

  // 2. Stop both PIDs
  //chPID.SetMode(chPID.Control::manual);
  hbPID.SetMode(hbPID.Control::manual);

  // 3. Flip the relay
  digitalWrite(PELTIER_REV_PIN, !inHeatMode);
  delay(80);  // Give relay time to settle

  // 4. Set correct direction for PIDs
  //chPID.SetControllerDirection(inHeatMode ? chPID.Action::direct : chPID.Action::reverse);
  hbPID.SetControllerDirection(inHeatMode ? hbPID.Action::direct : hbPID.Action::reverse);

  // 5. Load the correct P/I/D values for this mode + useOuterI state
  loadPIDs();

  hbPID.Reset();


  // 6. Restart the PIDs
  //chPID.SetMode(chPID.Control::automatic);
  hbPID.SetMode(hbPID.Control::automatic);
}
//set the heatsink fan speed.
void setFanSpeed() {
  uint8_t target = constrain(pwmDrive, 35, 100);

  const uint8_t rampRate = 1;  // Same rate up and down

  if (target > fanSpeed) {
    fanSpeed += rampRate * 3;
    if (fanSpeed > target) fanSpeed = target;
  } else if (target < fanSpeed) {
    fanSpeed -= rampRate;
    if (fanSpeed < target) fanSpeed = target;
  }

  SET_FAN(fanSpeed);
  //OCR1A = (uint8_t)(fanSpeed * 2.55f + 0.5f);
}

//function to handle things every minute, or 5 minutes.
void minutesFunctions(unsigned long now) {
  if (modeState == FRUIT) {
    humiditySetpoint = px[FRUIT_HUMIDITY_SETp];
    setpoint = px[FRUIT_TEMPp];
  } else {
    humiditySetpoint = px[INCUBATE_HUMIDITY_SETp];
    setpoint = px[INCUBATE_TEMPp];
  }
  if (humidity < humiditySetpoint) {
    humidifierRunningTimer = now;
    humidifierRunning = true;
    humidifierRan = 1;
  }
  if (humidity > 98) {
    airPumpRunningTimer = now;
    airPumpRunning = true;
  }
}

//function to handle things every hour
void hourFunctions(unsigned long now) {
  airPumpRunningTimer = now;
  airPumpRunning = true;
  setSyncProvider(getRtcTime);  // get the time from the RTC
}

//turn the lighting on/off and set intensity
void lightControl(bool state) {
  if (state) {

    digitalWrite(LIGHT_PIN1, bitRead(int(px[LIGHT_LEVELp]), 0));
    digitalWrite(LIGHT_PIN2, bitRead(int(px[LIGHT_LEVELp]), 1));
    digitalWrite(LIGHT_PIN3, bitRead(int(px[LIGHT_LEVELp]), 2));
    digitalWrite(LIGHT_PIN4, bitRead(int(px[LIGHT_LEVELp]), 3));
  } else {
    digitalWrite(LIGHT_PIN1, LOW);
    digitalWrite(LIGHT_PIN2, LOW);
    digitalWrite(LIGHT_PIN3, LOW);
    digitalWrite(LIGHT_PIN4, LOW);
  }
}

//handles button presses
void onEb1Clicked(EncoderButton& eb) {
  //Serial.println(F(">>> CALLBACK FIRED <<<"));
  if (displayMode == RUN) {      // if home secreen, launch menu 
       displayMode = MENU;
  } else {      //in not on home screen, pass the button press on to other functions
    button = 1;
    //if not running set switchStatus to button press for start condition
  }
}
// A function to handle the 'encoder' event
void onEb1Encoder(EncoderButton& eb) {  //this handles the button rotation
  encoder += eb.increment();
}

//handles the parameter programming menu
void programmingMenu(void) {  //this is a recycled mess. But it works nice.  GROK tried to rewrite it, but after hours it still didn't work. Speggeti code warning

  if (edit && menuPointer == 10) {    //are we selecting the parameters
    currentParam += encoder;          //change pointer of parameter by encoder counts
    if (currentParam < 0) {           //did we hit bottom of list?
      currentParam = SAVE_TO_EEPROM;  // if so jump to top
    }
    if (currentParam > SAVE_TO_EEPROM) {  //did we hit top of list?
      currentParam = 0;                   //if so jump to bottom
    }
  }
  if (lastPointer != currentParam && currentParam > SET_YEARp && currentParam != SERIAL_BAUD) {
    if (px[currentParam] == 0) {
      menuYes = 0;
    } else {
      menuYes = 1;
    }
  }

  if (lastPointer != currentParam && currentParam == SERIAL_BAUD) {
    while (px[SERIAL_BAUD] != baudRates[currentBaudPointer]) {
      currentBaudPointer++;
      if (currentBaudPointer > 14) {
        currentBaudPointer = 0;
        break;
      }
    }
  }
  lightControl(true);  //**********************FOR TESTING****************************

  lastPointer = currentParam;

  if (!edit && currentParam < SERIAL_BAUD) {  //are we in edit paramter values mode?
    menuPointer -= encoder;                   //move pointer by encoder counts
    if (menuPointer > 10) {                   //next 4 handle upwards rollover
      menuPointer = 0;
    } else if (menuPointer < 0) {
      menuPointer = 10;
    }
  } else if (!edit && currentParam > 10) {    //is cursor in special mode?  I BROKE THIS
    if (encoder != 0 && menuPointer == 10) {  //next 4 handle downwards  rollover
      menuPointer = 7;
    } else if (encoder != 0) {
      menuPointer = 10;
    }
  }

  if (currentParam < SERIAL_BAUD && edit && menuPointer != 10) {  //Is pointer inside the parameter value?                                      //not in special functions mode
    double Z = 0;
    switch (menuPointer) {        //load Z with proper value to add or
      case 0: Z = 1.0e-3; break;  //subtract from selected place
      case 1: Z = 1.0e-2; break;  //ie +/- .001 to 1000000
      case 2: Z = 1.0e-1; break;
      case 3: Z = 1.0e0; break;
      case 4: Z = 1.0e1; break;
      case 5: Z = 1.0e2; break;
      case 6: Z = 1.0e3; break;
      case 7: Z = 1.0e4; break;
      case 8: Z = 1.0e5; break;
      case 9: Z = 1.0e6; break;
      default: Z = 0;
    }
    if (currentParam == SET_DAYp || currentParam == SET_MONTHp || currentParam == SET_YEARp) {
      uint8_t maxD = maxDayInMonth((uint8_t)px[SET_MONTHp], (uint16_t)px[SET_YEARp]);
      px[SET_DAYp] = constrain(px[SET_DAYp], 1, maxD);
    }
    px[currentParam] = constrainValue(currentParam, px[currentParam]);
    px[currentParam] += (encoder * Z);  // +/- encoder counts from value
    if (px[currentParam] < 0) {         //catch negative parameter value
      px[currentParam] = 0;             //if so zero it out
    }

  } else if (currentParam >= SERIAL_BAUD && edit && menuPointer != 10) {  //are we in the  I BROKE THIS
    switch (currentParam) {                                               //figure out where we are and act accordanly

      case SERIAL_BAUD:                 //serial speed toggle through real values
        currentBaudPointer -= encoder;  //move pointer by encoder counts
        if (currentBaudPointer > 13) {  //  handle rollover
          currentBaudPointer = 0;
        } else if (currentBaudPointer < 0) {
          currentBaudPointer = 13;
        }
        px[SERIAL_BAUD] = baudRates[currentBaudPointer];
        if (button) {
          button = 0;
          menuPointer = 10;  // move selector back to parameters
          edit = 1;          // sw back to edit mode
          DEBUG_BEGIN(px[SERIAL_BAUD]);
        };
        break;

      case EXIT:  //exit menu
        if (encoder != 0) {
          if (menuYes == 1) {
            menuYes = 0;
          } else {
            menuYes = 1;
          }
        }
        if (button) {
          px[EXIT] = menuYes;
          button = 0;
          edit = 0;
        };
        break;

      case DUMP_TO_SERIAL:  //dump to serial
        if (encoder != 0) {
          if (menuYes == 1) {
            menuYes = 0;
          } else {
            menuYes = 1;
          }
        }
        if (button) {
          px[DUMP_TO_SERIAL] = menuYes;
          button = 0;
          menuPointer = 10;  // move selector back to parameters
          edit = 1;          // sw back to edit mode
        };
        break;

      case LOAD_DEFAULTS:  //load default parameters
        if (encoder != 0) {
          if (menuYes == 1) {
            menuYes = 0;
          } else {
            menuYes = 1;
          }
        }
        if (button) {
          px[LOAD_DEFAULTS] = menuYes;
          button = 0;
          menuPointer = 10;  // move selector back to parameters
          edit = 1;          // sw back to edit mode
        };
        break;

      case SAVE_TO_EEPROM:  //save to eeprom
        if (encoder != 0) {
          if (menuYes == 1) {
            menuYes = 0;
          } else {
            menuYes = 1;
          }
        }
        if (button) {
          px[SAVE_TO_EEPROM] = menuYes;
          button = 0;
          menuPointer = 10;  // move selector back to parameters
          edit = 1;          // sw back to edit mode
        };
    }
  }



  if (button) {  //swap between moving pointer and editing values
    edit = !edit;
  }

  button = 0;   //clear button press
  encoder = 0;  //clear encoder count


  if (px[EXIT] != 0) {  //exit edit mode
    px[EXIT] = 0;
    swapPIDmode();
    displayMode = RUN;  //exit programming
    runProgramCode = false;
    menuPointer = 10;  // move selector back to parameters
    edit = 0;
    lastPointer = 55;  //set to out of range to trip rereading for YES/NO
    px[EXIT] = 0;
  }

  if (px[DUMP_TO_SERIAL] != 0) {  //this dumps eemprom to serial port
    double x;
    for (int i = 0; i <= SAVE_TO_EEPROM; i++) {
      DEBUG_PRINT(F("Parameter[ "));
      DEBUG_PRINT(i);
      DEBUG_PRINT(F("] Name is "));
      char* currentStringPointer = (char*)pgm_read_word(&(parameterNames[i]));
      DEBUG_PRINT(currentStringPointer);
      //DEBUG_PRINT(parameterNames[i]);
      DEBUG_PRINT(F(" parameter data "));
      DEBUG_PRINT(px[i], 3);  //was truncating .025 to .03 fix not tested
      DEBUG_PRINT(F(" eeprom data "));
      EEPROM.get(EEPROM_STORAGE_ADDRESS + (i * 4), x);
      DEBUG_PRINTLN(x, 3);  //was truncating .025 to .03 fix
    }
    px[DUMP_TO_SERIAL] = 0;
    lastPointer = 55;  //set to out of range to trip rereading for YES/NO
  }

  if (px[LOAD_DEFAULTS] != 0) {     //load default parameters
    for (int i = 0; i < 39; i++) {  //clear the commands parameters
      px[i] = pgm_read_float(&(defaultpx[i]));
    }
    swapPIDmode();
    DEBUG_BEGIN(px[SERIAL_BAUD]);
    px[LOAD_DEFAULTS] = 0;
  }

  if (px[SAVE_TO_EEPROM] != 0) {                            //write parameters to eeprom
    for (int i = 0; i < SAVE_TO_EEPROM; i++) {              //this loop loads eeprom
      EEPROM.put(EEPROM_STORAGE_ADDRESS + (i * 4), px[i]);  //this loads eeprom from parameters
    }
    updateRTC();

    px[SAVE_TO_EEPROM] = 0;
    lastPointer = 55;  //set to out of range to trip rereading for YES/NO
  }
}

//handles the main menu
void menuMenu(void) {
  menuSelector += encoder;
  encoder = 0;
  if (menuSelector <= 0) {
    menuSelector = 1;
  }
  if (menuSelector >= 5) {
    menuSelector = 4;
  }
  if (button) {
    if (menuSelector == 1) {  //select fruiting
      modeState = FRUIT;
      EEPROM.update(PROFILE_TYPE_ADDRESS, 1);
      setpoint = px[FRUIT_TEMPp];
      humiditySetpoint = px[FRUIT_HUMIDITY_SETp];
      displayMode = RUN;
    }
    if (menuSelector == 2) {  //select incubatee
      modeState = INCUBATE;
      EEPROM.update(PROFILE_TYPE_ADDRESS, 0);
      setpoint = px[INCUBATE_TEMPp];
      humiditySetpoint = px[INCUBATE_HUMIDITY_SETp];
      displayMode = RUN;
    }
    if (menuSelector == 3) {  //select detailed status mode
      //regenSensorStart(1);    // *****************testing remove, unblock code below. ******
      //displayMode = RUN;
      displayMode = STATUS;
      StatusModeStarted = true;
      u8x8.clearDisplay();  //we are going to try to not clear the display in the status updates, removes flicker
      statusScreenTimer = millis();
    }
    if (menuSelector == 4) {        //select programming mode
      setSyncProvider(getRtcTime);  // get the time from the RTC
      setHours = px[SET_HOURSp] = hour();
      setMinutes = px[SET_MINUTESp] = minute();
      setMonth = px[SET_MONTHp] = month();
      setDay = px[SET_DAYp] = day();
      setYear = px[SET_YEARp] = year();

      displayMode = PROGRAM;
    }
    button = 0;
    runMenuCode = 0;
  }
}

//keeps variable values within bounds while parameters are being edited.
double constrainValue(uint8_t param, double val) {
  switch (param) {
    case DEAD_ZONEp: return constrain(val, 0, 20);
    case INCUBATE_TEMPp: return constrain(val, 50, 100);
    case FRUIT_TEMPp: return constrain(val, 50, 100);
    case INCUBATE_HUMIDITY_SETp: return constrain(val, 0, 96);
    case FRUIT_HUMIDITY_SETp: return constrain(val, 50, 97);
    case LIGHT_ON_HOURSp: return constrain(val, 0, 23.75);
    case WATER_SECONDSp: return constrain(val, 0, 10);
    case AIR_SECONDSp: return constrain(val, 1, 600);
    case PID_KP_HEATp: return constrain(val, 0, 1000);
    case PID_KI_HEATp: return constrain(val, 0, 1000);
    case PID_KD_HEATp: return constrain(val, 0, 1000);
    case PID_KP_COOLp: return constrain(val, 0, 1000);
    case PID_KI_COOLp: return constrain(val, 0, 1000);
    case PID_KD_COOLp: return constrain(val, 0, 1000);
    case SET_HOURSp: return constrain((int)val, 0, 23);
    case SET_MINUTESp: return constrain((int)val, 0, 59);
    case SET_MONTHp: return constrain((int)val, 1, 12);
    case SET_YEARp: return constrain((int)val, 2000, 2099);
    case LIGHT_LEVELp: return constrain((int)val, 0, 15);
    case STATUS_SECONDSp: return constrain(val, 5, 900);
    default: return max(val, 0.0);
  }
}

// Separate validation function – returns max day for a given month/year
uint8_t maxDayInMonth(uint8_t month, uint16_t year) {
  if (month < 1 || month > 12) return 31;  // safe fallback

  const uint8_t daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  uint8_t maxDay = daysInMonth[month - 1];

  if (month == 2) {
    bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
    if (leap) maxDay = 29;
  }
  return maxDay;
}

// Minimal conversion to save RAM
uint8_t decToBcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

uint8_t bcdToDec(uint8_t val) {
  return ((val / 16 * 10) + (val % 16));
}

time_t getRtcTime() {
  selectI2CChannel(I2C_CH_RTC);
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return 0;  // Hardware fail check

  Wire.requestFrom(0x68, 7);
  if (Wire.available() < 7) return 0;

  tmElements_t tm;
  tm.Second = bcdToDec(Wire.read() & 0x7F);
  tm.Minute = bcdToDec(Wire.read());
  tm.Hour = bcdToDec(Wire.read() & 0x3F);
  Wire.read();  // Skip Day of Week (0x03)
  tm.Day = bcdToDec(Wire.read());
  tm.Month = bcdToDec(Wire.read());
  // Convert 2-digit year to Y2K format for TimeLib
  tm.Year = CalendarYrToTm(bcdToDec(Wire.read()) + 2000);

  return makeTime(tm);
}

void updateRTC() {  //see if we need to update the RTC then update timeLib and
  bool minutesChanged = (setMinutes != px[SET_MINUTESp]);
  bool dataChanged = (setHours != px[SET_HOURSp] || setDay != px[SET_DAYp] || setMonth != px[SET_MONTHp] || setYear != px[SET_YEARp]);

  if (minutesChanged || dataChanged) {
    // read current RTC time first
    tmElements_t tm;
    breakTime(getRtcTime(), tm);

    if (minutesChanged) {

      tm.Minute = (uint8_t)px[SET_MINUTESp];
      tm.Second = 0;
    }
    if (dataChanged) {
      tm.Hour = (uint8_t)px[SET_HOURSp];
      tm.Day = (uint8_t)px[SET_DAYp];
      tm.Month = (uint8_t)px[SET_MONTHp];
      tm.Year = CalendarYrToTm((uint16_t)px[SET_YEARp]);
    }

    time_t newTime = makeTime(tm);
    setTime(newTime);
    writeFullRtcFromSysTime();  //then update the RTC
  }
}

void writeFullRtcFromSysTime() {  //load timeLib data into RTC
  selectI2CChannel(I2C_CH_RTC);
  Wire.beginTransmission(0x68);
  Wire.write(0x00);         // start at seconds
  Wire.write(decToBcd(0));  // seconds = 0
  Wire.write(decToBcd((uint8_t)minute()));
  Wire.write(decToBcd((uint8_t)hour()));
  Wire.write(0);  // day of week (optional)
  Wire.write(decToBcd((uint8_t)day()));
  Wire.write(decToBcd((uint8_t)month()));
  Wire.write(decToBcd((uint8_t)year() - 2000));  // 2-digit year
  Wire.endTransmission();
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
