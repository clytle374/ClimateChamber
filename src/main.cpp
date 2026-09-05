/* climate chamber controller.  Using DF robot temp and humidity sensor. cheap
128x64 6 pin spi u8x8 display. NTC.  Cheap ultrasonic huidifier element.  Some
white LEDs for lighting Started out with one of those pelita junction 6 can
refrigators. It's designed to grow small batches of delicious mushrooms, use it
for whatever. Code has been snipped, rewritten, modded, rewritten and modified
again from examples across the internet, and many of them recycled beyond
tracing on several of my project.  Libaries have licenses.  GROK has both helped
amd screwed me. Use it for whatever

************************************** TOO DO
LIST********************************* count I2C faults count sensor faults


night day temp and humidity cycles.
CATCH MOSFET RUNAWAY, safe system, pet watchdog.
***above mostly done, need to add eeprom lockout

make sure the relay defualts to cool mode in relaxed state

What an NTC failure actually looks like on your divider (10k pull-up, NTC to
GND):  DONE: BUT TEST THIS


Catch humidity readings way over setpoint and regen sensors. 

*/

#include "main.h"
#include "debug.h"
#include "display.h"
#include "sensors.h"
#include "utils.h"

float temperature;       // I2C temp calculated
float humidity;          // I2C huidity calculated
float NTCtempHeatblock;  // where we put NTC reading
float NTCtempHeatsink;   // where we put NTC reading
float humiditySetpoint;  // target temp, moved here due to status display
float ambientTemp = 100; // MB ambient temp  I hope they are close
float heatblockLastTemp; // for mosfet short detection
double px[41];           // parameters
int encoder;             // stored encoder couunts
bool button = 0;         // is button pressed?
int currentParam = 0;    // what is the current position of the programming menu
int lastPointer;         // save state to catch change
int menuPointer = 10;    // move selector to parameters.  10 is for changing
                         // selected parameter currentParam 0-9 pick a digit
bool edit = 1;           // sw back to edit mode
int menuYes = 0; // IDK, look into this  I thinkg this triggers the YES NO
                 // display in program memory
int currentBaudPointer =
    0; // pointer for selecting baudrates from list, should be local?
// control timer variables
unsigned long controlTimer;             // control internal time
unsigned long sensorTimer;              // time for reads
unsigned long statusScreenTimer;        // pid loop time
unsigned long humidifierRunningTimer;   // when did we start the humidifier
unsigned long airPumpRunningTimer;      // when did we start the airpump
unsigned long previousMillis = 0;       // Timestamp of the last execution
unsigned long dynamicInterval = 1000UL; // The current interval duration
int lastMinute = 0;  // keep track of the last run on minutes scheduler
int lastHour = 25;   // keep track of the last hour function run
int last6thHour = 1; // keep track of the time for the last 4 pre day cycle
int setYear;         // year for setting parameters
int setMonth;        // month for setting parameters
int setDay;          // day for setting parameters
int setHours;        // hours for setting parameters
int setMinutes;      // minutes for setting parameters.
bool airPumpRunning = false; // airpump
bool humidifierRunning = false;
uint8_t fanSpeed = 100;  // current actual speed (global or static)
uint8_t fanTarget = 25;  // Desired fan speed (updated from main control logic)
bool inHeatMode = true;  // PID to setup for heat
bool runProgramCode = 0; // switch on the programming code
bool runMenuCode = 0;    // switch on the menufunctions code
bool StatusModeStarted = 0; // are we going to be in status code?
int menuSelector = 3;       // selecting menu options
// bool humidifierRan = 0; // just for serial output as humidifer runs for a
// time
//  shorter than update.
bool useOuterI = false; // are we inhibiting the PID I term to stop the endless
                        // windup problem
// screen scrolling variables.
int screenX = 0; // locations for display scanning to not burn screen
int screenY = 0; // locations for display scanning to not burn screen
int xShift;      // used to return a variable from a right justify function
// ***** PID CONTROL VARIABLES *****
float setpoint; // target temp
// double input = 21;    //zero will throw errors before first averaged sample
float pwmDrive;       // output from PID for main element
float heatBlockInput; // output for the cool PID
float PID1output; // we're going to save the output of PID1 here and use it for
                  // deltaT and serial debugging
bool ntcBlockRawOk = true; // is the NTC acting right
bool ntcSinkRawOk = true;  // is the NTC acting right
bool systemIsHalted =
    false; // if either NTC is acting up, shutdown peltier and run fan
float heatblockDeltaT =
    0; // for tracking the delta T of the heatblock to catch mosfet shorted.
unsigned long noSensorSince = 0; // timer to start if e have no sensor data
bool noSensorFault = false;      // fault state for no sensor data.
uint16_t numberOfWireFaults =
    0; // logging the number of wire timeouts, crash hunting

// setup running averages
RunningAverage AMBIENT_TEMP(5);
RunningAverage HEATBLOCK_TEMP(5);
RunningAverage HEATSINK_TEMP(5);
RunningAverage PWM_DRIVE(2);
// these are for the 4 SHT41A sensors
RunningAverage CHAMBER_TEMP[4] = {RunningAverage(5), RunningAverage(5),
                                  RunningAverage(5), RunningAverage(5)};
RunningAverage CHAMBER_HUM[4] = {RunningAverage(5), RunningAverage(5),
                                 RunningAverage(5), RunningAverage(5)};
SensorState sensor[4]; // sensor[0] through sensor[4]

displayMode_t displayMode;
modeState_t modeState;

// these are the texts for the menu.
const char menuOptions[][4] = {"NO", "YES"}; // display for ON/OFF menu options
// add the names of the 12 months to output to the OLED
const char *const monthNames[] PROGMEM = {"Jan", "Feb", "Mar", "Apr",
                                          "May", "Jun", "Jul", "Aug",
                                          "Sep", "Oct", "Nov", "Dec"};
// name all the paramaters for the programming menu
const char *const parameterNames[] PROGMEM = {
    "DEAD_ZONE",

    "INCUBATE_TEMP",  "INCUBATE_HUMID", "FRUIT_TEMP",    "FRUIT_HUMID",
    "LIGHT_ON_HOURS", "LIGHT_LEVEL",    "WATER_SECONDS", "AIR_SECONDS",
    "CH_P_HEAT",      "CH_I_HEAT",      "CH_D_HEAT",     "BL_P_HEAT",
    "CH_P_COOL",      "CH_I_COOL",      "CH_D_COOL",     "BL_P_COOL",
    "STATUS_SEC",     "SET_HOUR",       "SET_MINUTE",    "SET_MONTH",
    "SET_DAY",        "SET_YEAR",       "SERIAL_SPEED",  "EXIT",
    "DUMP_TO_SERIAL", "LOAD_DEFAULTS",  "SAVE"};

// these are the defaults for the parameters. I think?  GD am a ture
const float defaultpx[30] PROGMEM = {DEAD_ZONE,
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
                                     0};

// availble baud rates, did I miss anything important?
const float baudRates[14] = {2400,   4800,   9600,   14400,  19200,
                             28800,  38400,  57600,  74800,  115200,
                             230400, 250000, 500000, 1000000};

//***********************stopped switching move to header file here */
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

EncoderButton eb1(ROTARY_PIN1, ROTARY_PIN2,
                  BUTTON_PIN); // sets up the encoder buton
// setup the 2 PID functions, in cascade
// Heat/cool and paramters are switched when switching HEAT/COOL function
QuickPID chPID(&temperature, &PID1output, &setpoint);          // outer loop
QuickPID hbPID(&NTCtempHeatblock, &pwmDrive, &heatBlockInput); // inner loop

void setup() {
  MCUSR = 0;     // clearing watchdog
  wdt_disable(); // disable the watchdog
  //  *********************TESTING****************
  displayMode =
      RUN; // put display in run to start out      *************************
  pinMode(ROTARY_PIN1, INPUT_PULLUP); //************************added are they
                                      // needed?*******************
  pinMode(ROTARY_PIN2, INPUT_PULLUP); //************************added are they
                                      // needed?*******************
  pinMode(BUTTON_PIN, INPUT_PULLUP);  //************************added are they
                                      // needed?*******************
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
  pinMode(PELTIER_ON_PIN, OUTPUT); // PD7

  // Timer 2: Fast PWM, non-inverting
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS20); // Prescaler 1 → ~78 kHz
  OCR2A = 0;            // Start at 0%

  // === Fan PWMs ===
  pinMode(FAN_PWM, OUTPUT); // PD5 as output (OC1A)
                            // Timer 1: 8-bit Fast PWM, non-inverting on OC1A
  TCCR1A = (1 << COM1A1) | (1 << WGM10); // WGM11=0, WGM10=1  → 8-bit mode
  TCCR1B = (1 << WGM12) | (1 << CS11);   // WGM12=1, prescaler 8

  OCR1A = 255; // Start at 100%

  for (int i = 0; i < EXIT; i++) { // this loop loads eeprom into parameters
    // EEPROM.put(EEPROM_STORAGE_ADDRESS + (i*4) , px[i]);   //this loads eeprom
    // from parameters use this for new baord  ******doesn't work******
    EEPROM.get(EEPROM_STORAGE_ADDRESS + (i * 4),
               px[i]); // this loads parameters from eeprom
  }
  for (int i = EXIT; i < SAVE_TO_EEPROM; i++) { // clear the commands parameters
    px[i] = 0;
  }

  unsigned char value; // = EEPROM.read(PROFILE_TYPE_ADDRESS);
  EEPROM.get(PROFILE_TYPE_ADDRESS,
             value); // resume last state in case of power fail
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
    sensor[i].faultCounter = 0;
  }

  // DEBUG_BEGIN(57600); // set serial speed
  DEBUG_BEGIN(px[SERIAL_BAUD]); // set serial speed
  DEBUG_PRINTLN("hello");

  // encoder switch events
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  Wire.begin();
  Wire.setWireTimeout(25000,
                      true); // 25 ms timeout, reset TWI hardware on timeout
  u8x8.begin();              // display startup
                             // u8x8.setPowerSave(0);
  u8x8.setContrast(50);      // turned down for display life
  u8x8.setFlipMode(0);       // Flip the display 180 degrees

  // clear running averages
  for (int i = 0; i < 4; i++) {
    CHAMBER_TEMP[i].clear();
    CHAMBER_HUM[i].clear();
  }
  AMBIENT_TEMP.clear();
  HEATBLOCK_TEMP.clear();
  HEATSINK_TEMP.clear();
  PWM_DRIVE.clear();

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
  chPID.SetAntiWindupMode(chPID.iAwMode::iAwClamp); // anti-windup control
  hbPID.SetAntiWindupMode(hbPID.iAwMode::iAwClamp); // anti-windup control

  updateUseOuterI(); // Set the initial state of useOuterI based on current
                     // error
  swapPIDmode();     // set the
  loadPIDs();        // loads the P,I,D values into the PIDs

  selectI2CChannel(I2C_CH_RTC);
  setSyncProvider(getRtcTime);
  lightControl(false);
  for (int i = 0; i < 5; i++) {
    sampleSensors();
    delay(100);
  }
  DEBUG_PRINT(F("SPHUM="));
  DEBUG_PRINTLN(humiditySetpoint);
  DEBUG_PRINT(F("SPTEMP="));
  DEBUG_PRINTLN(setpoint);
  DEBUG_PRINT(F("HEATMODE="));
  DEBUG_PRINTLN(inHeatMode);
  DEBUG_PRINTLN(F("S0VALID=1, S1VALID=1, S2VALID=1, S3VALID=1"));
  DEBUG_PRINTLN(F("S0REGEN=0, S1REGEN=0, S2REGEN=0, S3REGEN=0"));
  DEBUG_PRINTLN(F("S0FAILED=0, S1FAILED=0, S2FAILED=0, S3FAILED=0"));

  wdt_enable(WDTO_4S); // arm the watchdog
}

void loop() { //******************main loop************************
  unsigned long currentMillis = millis(); // Get the current time
  eb1.update();                           // button update
  wdt_reset();                            // Good doggy

  if (!systemIsHalted) { // don't run PIDs, we have something not okay.
    //  Ask if outer PID loop is ready to execute
    if (chPID.Compute()) { // if PID1 runs, its timebase is controlled
                           // internally. Lets use the output to make a useable
                           // input for PID2
      float deltaT =
          PID1output * 0.33f; // lets scale PID1's output to 0 to X degrees F
                              // over the current chamber temp
      float ambientOffset = (setpoint - ambientTemp) *
                            0.1f; // feed forward the ambient vs setpoint in
                                  // increase control range on heatblock
      heatBlockInput = constrain(temperature + deltaT + ambientOffset, -10,
                                 140); // Aim for delta from process
    }
    // Ask if inner PID loop is ready to execute
    if (hbPID.Compute()) { // call PID2, if it returns that it ran, spit out
                           // debugging info

      //*********************
      // Convert to 8-bit value (0-255)
      uint8_t dutyCycle =
          (uint8_t)(pwmDrive * 2.55 + 0.5); // +0.5 for better rounding
      PWM_DRIVE.addValue(pwmDrive);
      dutyCycle = constrain(dutyCycle, 0, 245);
      SET_PWM(dutyCycle);
      DEBUG_PRINT(F("SYSHUM="));
      DEBUG_PRINTLN(humidity);
      DEBUG_PRINT(F("SYSTEMP="));
      DEBUG_PRINTLN(temperature);
      DEBUG_PRINT(F("PWM="));
      DEBUG_PRINTLN(pwmDrive);
      DEBUG_PRINT(F("FAN="));
      DEBUG_PRINTLN(fanSpeed);
      DEBUG_PRINT(F("PIDOUT="));
      DEBUG_PRINTLN(PID1output);
      DEBUG_PRINT(F("HTBLKT="));
      DEBUG_PRINTLN(NTCtempHeatblock);
      DEBUG_PRINT(F("HSINKT="));
      DEBUG_PRINTLN(NTCtempHeatsink);
      DEBUG_PRINT(F("AMBT="));
      DEBUG_PRINTLN(ambientTemp);
      DEBUG_PRINT(F("HTBLKINPUT="));
      DEBUG_PRINTLN(heatBlockInput);

      // DEBUG_FLUSH(); // Keeps the CPU here until the talk is done
    }
  }
  // sample sensors every 1000 millis
  if (currentMillis - sensorTimer >= SENSOR_RATE) {

    sensorTimer = currentMillis; // this is rolloever safe acording to AI
    sampleSensors();
    digitalWrite(RUN_LED, !digitalRead(RUN_LED));
    setFanSpeed();
  }
  // test if it's time to update the display
  if (currentMillis - previousMillis >= dynamicInterval) { // update DISPLAY
    previousMillis = currentMillis; // Get current time once
    displayUpdate();
  }
  // test if it's time to timeout the status display.  Don't burn the OLED
  if (StatusModeStarted == true &&
      currentMillis - statusScreenTimer >=
          (px[STATUS_SECONDSp] *
           1000)) { //*****lets setup a detailed status page display, we'll use
                    // this to time it out to not burn the
                    // screem****************
    displayMode = RUN;
    StatusModeStarted = false;
  }
  // run the general control loop every 5 seconds, 5000 miillis
  if (currentMillis - controlTimer >= CONTROL_RATE) { // run controls   5000
    controlTimer = currentMillis; // this is rolloever safe acording to AI
    // raise and lower the sun.
    double minutesNow = (hour() * 60) + minute();
    double lightMinutes =
        px[LIGHT_ON_HOURSp] * 30; //  number of minutes before and after noon
                                  //  light should be on hours * 60 / 2
    if ((minutesNow > 720 - lightMinutes && minutesNow < 720 + lightMinutes) &&
        modeState ==
            FRUIT) { // are we in fruit and is it time for lights?
                     // ************FIX THIS, LIGHT_TIME IGNORED**********
      lightControl(true);
    } else {
      lightControl(false);
    }
    // lets do some stuff every  MINUTE
    if (minute() != lastMinute) {
      lastMinute = minute();
      minutesFunctions(currentMillis);
      // DEBUG_PRINTLN(F("minute functions ran"));
    }
    // lets do some stuff ever hour
    if (hour() != lastHour) {
      lastHour = hour();
      hourFunctions(currentMillis);
      // DEBUG_PRINTLN(F("hour functions ran"));
    }
    // lets do some stuff every 6 hours, daily regen of sensors
    if (hour() != last6thHour && hour() % 6 == 0) {
      last6thHour = hour();
      SixHourFunctions();
    }
    // do some other things every 5 seconds CONTROL_RATE
    detectHCmode();
    updateUseOuterI();
    checkForMosfetFailure();
    //  setFanSpeed(40);
  }
  if (runMenuCode == true) { // code for changeing function withing the menu
    menuMenu();
  }
  if (runProgramCode == true) { // are we in programing mode?
    programmingMenu();
  }
  if (humidifierRunning &&
      (currentMillis - humidifierRunningTimer >=
       (px[WATER_SECONDSp] * 1000))) { // shutdown humidifier when time is up
    humidifierRunning = false;
    DEBUG_PRINTLN(F("HUMIDIFIERON=0"));
  }
  if (airPumpRunning &&
      (currentMillis - airPumpRunningTimer >=
       (px[AIR_SECONDSp] * 1000))) { // shutdown the airpump when time is up
    airPumpRunning = false;
    DEBUG_PRINTLN(F("AIRPUMPON=0"));
  }

  //  WRITE TO THE OUTPUTS NOw THAT WE'RE AT THE END
  digitalWrite(HUMIDIFIER_PIN, humidifierRunning);
  digitalWrite(AIRPUMP_PIN, airPumpRunning);
  // **********************  status LEDS *********************
  digitalWrite(H2O_LED, humidifierRunning);
  digitalWrite(AIR_LED, airPumpRunning);
  // regenMaint();  //check and see if a sensor is done with regeneration
}

// detect if we need to swap between heat and cool mode
void detectHCmode() {

  if (PID1output > 0) {
    // Temperature is too low, definitively needs heat
    if (inHeatMode == false) { // Only print/switch if changing state
      inHeatMode = true;
      DEBUG_PRINT(F("HEATMODE=1"));
      swapPIDmode();
    }
  }
  if (PID1output < 0) {
    // Temperature is too high, definitively needs cooling
    if (inHeatMode == true) { // Only print/switch if changing state
      inHeatMode = false;
      DEBUG_PRINT(F("HEATMODE=0"));
      swapPIDmode();
    }
  }
  digitalWrite(HEAT_LED, inHeatMode);
}

// detect if we are withing half of deadzone from target to change loop
// agressiveness
void updateUseOuterI() {
  static bool lastUseOuterI = false;
  static bool hasRan = false;
  float error = abs(setpoint - temperature);
  if (!hasRan) {
    if (error < px[DEAD_ZONEp] * 2) {
      useOuterI = true;
      DEBUG_PRINTLN(F("OUTERION=1"));
    } else {
      useOuterI = false;
      DEBUG_PRINTLN(F("OUTERION=0"));
    }
  } else {
    hasRan = true;
  }

  // Turn I ON when error gets small enough (with some hysteresis)
  if (!useOuterI && error < px[DEAD_ZONEp] * 2) {
    useOuterI = true;
    DEBUG_PRINTLN(F("OUTERION=1"));
  }
  // Turn I OFF only when error gets clearly larger
  if (useOuterI && error > px[DEAD_ZONEp] * 4) {
    useOuterI = false;
    DEBUG_PRINTLN(F("OUTERION=0"));
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
    kp = useOuterI ? px[PID_KP_HEATp] : px[PID_KP_HEATp] * 1.5f; // was 1.5,
    ki = useOuterI ? px[PID_KI_HEATp] : 0.0f;
    kd = px[PID_KD_HEATp];
    blockP = px[PID_P_BLOCK_HEATp];
  } else {
    kp = useOuterI ? px[PID_KP_COOLp] : px[PID_KP_COOLp] * 1.5f; // was 1.5,
    ki = useOuterI ? px[PID_KI_COOLp] : 0.0f;
    kd = px[PID_KD_COOLp];
    blockP = px[PID_P_BLOCK_COOLp];
  }

  chPID.SetTunings(kp, ki, kd);

  hbPID.SetTunings(blockP, 0, 0); // Inner loop stays simple
}

// handle swapping the inner loop direction and safely switching the relay
void swapPIDmode(void) {
  // 1. Kill the output hard (important with inverted PWM)
  SET_PWM(255);
  delay(100);

  // 2. Stop both PIDs
  // chPID.SetMode(chPID.Control::manual);
  hbPID.SetMode(hbPID.Control::manual);

  // 3. Flip the relay
  digitalWrite(PELTIER_REV_PIN, !inHeatMode);
  delay(80); // Give relay time to settle

  // 4. Set correct direction for PIDs
  // chPID.SetControllerDirection(inHeatMode ? chPID.Action::direct :
  // chPID.Action::reverse);
  hbPID.SetControllerDirection(inHeatMode ? hbPID.Action::direct
                                          : hbPID.Action::reverse);

  // 5. Load the correct P/I/D values for this mode + useOuterI state
  loadPIDs();

  hbPID.Reset();

  // 6. Restart the PIDs
  // chPID.SetMode(chPID.Control::automatic);
  hbPID.SetMode(hbPID.Control::automatic);
}
// set the heatsink fan speed.
void setFanSpeed() {

  const uint8_t rampRate = 1; // Same rate up and down
  uint8_t target;

  // find target fan speed. 100% for too hot/cold, halt, or useOuterI = false
  if (systemIsHalted || NTCtempHeatsink >= 120.0f || NTCtempHeatsink <= 32.0f ||
      !useOuterI) {
    target = 100;
  } else {
    float dT = fabs(NTCtempHeatsink - ambientTemp);
    target = 35 + (uint8_t)constrain(dT * (65.0f / 30.0f), 0, 65);
  }

  // ramp fanSpeed toward target
  if (target > fanSpeed) {
    fanSpeed += rampRate * 3;
    if (fanSpeed > target)
      fanSpeed = target;
  } else if (target < fanSpeed) {
    fanSpeed -= rampRate;
    if (fanSpeed < target)
      fanSpeed = target;
  }
  SET_FAN(fanSpeed);
}

// function to handle things every minute, or 5 minutes.
void minutesFunctions(unsigned long now) {
  static uint8_t stepIndex = 0;
  if (modeState == FRUIT) {
    humiditySetpoint = px[FRUIT_HUMIDITY_SETp];
    setpoint = px[FRUIT_TEMPp];
  } else {
    humiditySetpoint = px[INCUBATE_HUMIDITY_SETp];
    setpoint = px[INCUBATE_TEMPp];
  }
  DEBUG_PRINT(F("SPHUM="));
  DEBUG_PRINTLN(humiditySetpoint);
  DEBUG_PRINT(F("SPTEMP="));
  DEBUG_PRINTLN(setpoint);

  if (humidity < humiditySetpoint) {
    humidifierRunningTimer = now;
    humidifierRunning = true;
    DEBUG_PRINTLN(F("HUMIDIFIERON=1"));
    // humidifierRan = 1;
  }
  if (humidity > 98) {
    airPumpRunningTimer = now;
    airPumpRunning = true;
    DEBUG_PRINTLN(F("AIRPUMPON=1"));
  }
  switch (stepIndex) {
  case 0:
    sensorTempTest();
    sensorHumidityTest();
    break;
  case 1:
    sensorMaint();
    break;
  }
  stepIndex++;
  if (stepIndex >= 2) {
    stepIndex = 0;
  }
}

// function to handle things every hour
void hourFunctions(unsigned long now) {
  airPumpRunningTimer = now;
  airPumpRunning = true;
  DEBUG_PRINTLN(F("AIRPUMPON=1"));
  setSyncProvider(getRtcTime); // get the time from the RTC
}

// turn the lighting on/off and set intensity
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

// test if system seems mormal
void checkForMosfetFailure(void) {
  if (systemIsHalted) {
    return;
  }
  float heatblockDeltaT = heatblockLastTemp - NTCtempHeatblock;
  if (PWM_DRIVE.getAverage() < 5) {
    if (fabs(heatblockDeltaT) > 1) {
      DEBUG_PRINTLN(F("MOSFET FAILURE SUSPECTED"));
      confirmMosfetShort(); // are things are happening?
    }
  }
}

// confirm runaway condition
void confirmMosfetShort(void) {
  int faultCount = 60;
  SET_PWM(0); // stop the peltier drive
  while (1) {
    if ((inHeatMode && (NTCtempHeatblock > heatblockLastTemp)) ||
        (!inHeatMode && (NTCtempHeatblock < heatblockLastTemp))) {
      faultCount++;
      if (faultCount > 100) {
        DEBUG_PRINTLN(F("MOSFET FAILED"));
        scuttleShip();
      }
    } else {
      faultCount--;
      if (faultCount <= 20) {
        return;
      }
    }
    DEBUG_PRINT(F("MOSFET FAULT COUNT "));
    DEBUG_PRINTLN(faultCount);
    sampleSensors();
    wdt_reset(); // Good doggy
    delay(1000);
  }
}

// safe unit if peltier is running away
void scuttleShip(void) {
  // chPID.SetMode(chPID.Control::manual);
  // hbPID.SetMode(hbPID.Control::manual);
  SET_PWM(0); // PWM off (useless if FET shorted, still do it)
              // pwmDrive = 0;
  // if (inHeatMode) {
  digitalWrite(PELTIER_REV_PIN, LOW); // match whatever "cool" already is
  // inHeatMode = false;
  //}
  // if already cool, do not touch the relay

  SET_FAN(0);
  // fanSpeed = 0;
  // fanTarget = 0;

  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(AIRPUMP_PIN, LOW);
  digitalWrite(ERROR_LED, HIGH);

  for (;;) {
    wdt_reset();
    digitalWrite(ERROR_LED, !digitalRead(ERROR_LED));
    delay(500);
  }
}
