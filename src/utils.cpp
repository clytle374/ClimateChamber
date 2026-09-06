#include "utils.h"
#include "debug.h"
#include "display.h"
#include "main.h"
#include "sensors.h"

ISR(INT0_vect) { encStep(); } // encoder route setup
ISR(INT1_vect) { encStep(); } // both interupts call the same subrouting.

/*
//handles button presses
void onEb1Clicked(EncoderButton& eb) {
  //Serial.println(F(">>> CALLBACK FIRED <<<"));
  if (displayMode == RUN) {      // if home secreen, launch menu
       displayMode = MENU;
  } else {      //in not on home screen, pass the button press on to other
functions button = 1;
    //if not running set switchStatus to button press for start condition
  }
}
// A function to handle the 'encoder' event
void onEb1Encoder(EncoderButton& eb) {  //this handles the button rotation
  encoder += eb.increment();
}
*/

int8_t encoderRead(void) { // call this from loop()?  Why can't I just encDelta?
  int8_t d;
  noInterrupts();
  d = encDelta;
  encDelta = 0;
  interrupts();
  return d;
}

// handles encoder button into code
void EBupdate(void) {
  encoder += encoderRead(); // or inline the noInterrupts sandwich here

  static uint8_t lastBtn = 1; // pull-up, idle high
  static uint32_t lastMs = 0;
  uint8_t now = digitalRead(BUTTON_PIN);

  if (now != lastBtn && (millis() - lastMs) > 20) {
    lastMs = millis();
    lastBtn = now;
    if (now == 0) { // pressed
      if (displayMode == RUN)
        displayMode = MENU;
      else
        button = 1;
    }
  }
}

// setsup the encoder pins
void encoderSetup(void) {           // call this from setup?
  DDRD &= ~(_BV(PD2) | _BV(PD3));   // set as inputs?
  PORTD |= _BV(PD2) | _BV(PD3);     // pullups  turns them on?
  EICRA |= _BV(ISC00) | _BV(ISC10); // either edge   state change interupt
  EIMSK |= _BV(INT0) | _BV(INT1);   // sets them as interupts
}

// handles encoder events
void encStep(void) { // interupt sub routine
  static uint8_t last = 0;
  uint8_t now = (PIND & 0x0C) >> 2; // PD3:PD2  This is doing what?
  uint8_t sum = (last << 2) |
                now; // so this is bitshift left 2 places, and then OR with now?
  if (now == 0b11) { // detent rest (A and B high)
    if (sum == 0b1011)
      encDelta++; // arrived via CW
    else if (sum == 0b0111)
      encDelta--;
  }
  last = now; // save the current state
}

// handles the parameter programming menu
void programmingMenu(void) { // this is a recycled mess. But it works nice. GROK
                             // tried to rewrite it, but after hours it still
                             // didn't work. Speggeti code warning

  if (edit && menuPointer == 10) { // are we selecting the parameters
    currentParam += encoder; // change pointer of parameter by encoder counts
    if (currentParam < 0) {  // did we hit bottom of list?
      currentParam = SAVE_TO_EEPROM; // if so jump to top
    }
    if (currentParam > SAVE_TO_EEPROM) { // did we hit top of list?
      currentParam = 0;                  // if so jump to bottom
    }
  }
  if (lastPointer != currentParam && currentParam > SET_YEARp &&
      currentParam != SERIAL_BAUD) {
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
  lightControl(
      true); //**********************FOR TESTING****************************

  lastPointer = currentParam;

  if (!edit &&
      currentParam < SERIAL_BAUD) { // are we in edit paramter values mode?
    menuPointer -= encoder;         // move pointer by encoder counts
    if (menuPointer > 10) {         // next 4 handle upwards rollover
      menuPointer = 0;
    } else if (menuPointer < 0) {
      menuPointer = 10;
    }
  } else if (!edit &&
             currentParam > 10) { // is cursor in special mode?  I BROKE THIS
    if (encoder != 0 && menuPointer == 10) { // next 4 handle downwards rollover
      menuPointer = 7;
    } else if (encoder != 0) {
      menuPointer = 10;
    }
  }

  if (currentParam < SERIAL_BAUD && edit &&
      menuPointer != 10) { // Is pointer inside the parameter value? //not in
                           // special functions mode
    double Z = 0;
    switch (menuPointer) { // load Z with proper value to add or
    case 0:
      Z = 1.0e-3;
      break; // subtract from selected place
    case 1:
      Z = 1.0e-2;
      break; // ie +/- .001 to 1000000
    case 2:
      Z = 1.0e-1;
      break;
    case 3:
      Z = 1.0e0;
      break;
    case 4:
      Z = 1.0e1;
      break;
    case 5:
      Z = 1.0e2;
      break;
    case 6:
      Z = 1.0e3;
      break;
    case 7:
      Z = 1.0e4;
      break;
    case 8:
      Z = 1.0e5;
      break;
    case 9:
      Z = 1.0e6;
      break;
    default:
      Z = 0;
    }
    if (currentParam == SET_DAYp || currentParam == SET_MONTHp ||
        currentParam == SET_YEARp) {
      uint8_t maxD =
          maxDayInMonth((uint8_t)px[SET_MONTHp], (uint16_t)px[SET_YEARp]);
      px[SET_DAYp] = constrain(px[SET_DAYp], 1, maxD);
    }
    px[currentParam] = constrainValue(currentParam, px[currentParam]);
    px[currentParam] += (encoder * Z); // +/- encoder counts from value
    if (px[currentParam] < 0) {        // catch negative parameter value
      px[currentParam] = 0;            // if so zero it out
    }

  } else if (currentParam >= SERIAL_BAUD && edit &&
             menuPointer != 10) { // are we in the  I BROKE THIS
    switch (currentParam) {       // figure out where we are and act accordanly

    case SERIAL_BAUD:                // serial speed toggle through real values
      currentBaudPointer -= encoder; // move pointer by encoder counts
      if (currentBaudPointer > 13) { //  handle rollover
        currentBaudPointer = 0;
      } else if (currentBaudPointer < 0) {
        currentBaudPointer = 13;
      }
      px[SERIAL_BAUD] = baudRates[currentBaudPointer];
      if (button) {
        button = 0;
        menuPointer = 10; // move selector back to parameters
        edit = 1;         // sw back to edit mode
        DEBUG_BEGIN(px[SERIAL_BAUD]);
      };
      break;

    case EXIT: // exit menu
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

    case DUMP_TO_SERIAL: // dump to serial
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
        menuPointer = 10; // move selector back to parameters
        edit = 1;         // sw back to edit mode
      };
      break;

    case LOAD_DEFAULTS: // load default parameters
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
        menuPointer = 10; // move selector back to parameters
        edit = 1;         // sw back to edit mode
      };
      break;

    case SAVE_TO_EEPROM: // save to eeprom
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
        menuPointer = 10; // move selector back to parameters
        edit = 1;         // sw back to edit mode
      };
    }
  }

  if (button) { // swap between moving pointer and editing values
    edit = !edit;
  }

  button = 0;  // clear button press
  encoder = 0; // clear encoder count

  if (px[EXIT] != 0) { // exit edit mode
    px[EXIT] = 0;
    swapPIDmode();
    displayMode = RUN; // exit programming
    runProgramCode = false;
    menuPointer = 10; // move selector back to parameters
    edit = 0;
    lastPointer = 55; // set to out of range to trip rereading for YES/NO
    px[EXIT] = 0;
  }

  if (px[DUMP_TO_SERIAL] != 0) { // this dumps eemprom to serial port
    double x;
    for (int i = 0; i <= SAVE_TO_EEPROM; i++) {
      DEBUG_PRINT(F("Parameter[ "));
      DEBUG_PRINT(i);
      DEBUG_PRINT(F("] Name is "));
      char *currentStringPointer = (char *)pgm_read_word(&(parameterNames[i]));
      DEBUG_PRINT(currentStringPointer);
      // DEBUG_PRINT(parameterNames[i]);
      DEBUG_PRINT(F(" parameter data "));
      DEBUG_PRINT(px[i], 3); // was truncating .025 to .03 fix not tested
      DEBUG_PRINT(F(" eeprom data "));
      EEPROM.get(EEPROM_STORAGE_ADDRESS + (i * 4), x);
      DEBUG_PRINTLN(x, 3); // was truncating .025 to .03 fix
    }
    px[DUMP_TO_SERIAL] = 0;
    lastPointer = 55; // set to out of range to trip rereading for YES/NO
  }

  if (px[LOAD_DEFAULTS] != 0) {    // load default parameters
    for (int i = 0; i < 39; i++) { // clear the commands parameters
      px[i] = pgm_read_float(&(defaultpx[i]));
    }
    swapPIDmode();
    DEBUG_BEGIN(px[SERIAL_BAUD]);
    px[LOAD_DEFAULTS] = 0;
  }

  if (px[SAVE_TO_EEPROM] != 0) {               // write parameters to eeprom
    for (int i = 0; i < SAVE_TO_EEPROM; i++) { // this loop loads eeprom
      EEPROM.put(EEPROM_STORAGE_ADDRESS + (i * 4),
                 px[i]); // this loads eeprom from parameters
    }
    updateRTC();

    px[SAVE_TO_EEPROM] = 0;
    lastPointer = 55; // set to out of range to trip rereading for YES/NO
  }
}

// handles the main menu
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
    if (menuSelector == 1) { // select fruiting
      modeState = FRUIT;
      EEPROM.update(PROFILE_TYPE_ADDRESS, 1);
      setpoint = px[FRUIT_TEMPp];
      humiditySetpoint = px[FRUIT_HUMIDITY_SETp];
      displayMode = RUN;
    }
    if (menuSelector == 2) { // select incubatee
      modeState = INCUBATE;
      EEPROM.update(PROFILE_TYPE_ADDRESS, 0);
      setpoint = px[INCUBATE_TEMPp];
      humiditySetpoint = px[INCUBATE_HUMIDITY_SETp];
      displayMode = RUN;
    }
    if (menuSelector == 3) { // select detailed status mode
      // regenSensorStart(1);    // *****************testing remove, unblock
      // code below. ****** displayMode = RUN;
      displayMode = STATUS;
      StatusModeStarted = true;
      u8x8.clearDisplay(); // we are going to try to not clear the display in
                           // the status updates, removes flicker
      statusScreenTimer = millis();
    }
    if (menuSelector == 4) {       // select programming mode
      setSyncProvider(getRtcTime); // get the time from the RTC
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

// keeps variable values within bounds while parameters are being edited.
double constrainValue(uint8_t param, double val) {
  switch (param) {
  case DEAD_ZONEp:
    return constrain(val, 0, 20);
  case INCUBATE_TEMPp:
    return constrain(val, 50, 100);
  case FRUIT_TEMPp:
    return constrain(val, 50, 100);
  case INCUBATE_HUMIDITY_SETp:
    return constrain(val, 0, 96);
  case FRUIT_HUMIDITY_SETp:
    return constrain(val, 50, 97);
  case LIGHT_ON_HOURSp:
    return constrain(val, 0, 23.75);
  case WATER_SECONDSp:
    return constrain(val, 0, 10);
  case AIR_SECONDSp:
    return constrain(val, 1, 600);
  case PID_KP_HEATp:
    return constrain(val, 0, 1000);
  case PID_KI_HEATp:
    return constrain(val, 0, 1000);
  case PID_KD_HEATp:
    return constrain(val, 0, 1000);
  case PID_KP_COOLp:
    return constrain(val, 0, 1000);
  case PID_KI_COOLp:
    return constrain(val, 0, 1000);
  case PID_KD_COOLp:
    return constrain(val, 0, 1000);
  case SET_HOURSp:
    return constrain((int)val, 0, 23);
  case SET_MINUTESp:
    return constrain((int)val, 0, 59);
  case SET_MONTHp:
    return constrain((int)val, 1, 12);
  case SET_YEARp:
    return constrain((int)val, 2000, 2099);
  case LIGHT_LEVELp:
    return constrain((int)val, 0, 15);
  case STATUS_SECONDSp:
    return constrain(val, 5, 900);
  default:
    return max(val, 0.0);
  }
}

// Separate validation function – returns max day for a given month/year
uint8_t maxDayInMonth(uint8_t month, uint16_t year) {
  if (month < 1 || month > 12)
    return 31; // safe fallback

  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
  uint8_t maxDay = daysInMonth[month - 1];

  if (month == 2) {
    bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
    if (leap)
      maxDay = 29;
  }
  return maxDay;
}

// Minimal conversion to save RAM
uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

uint8_t bcdToDec(uint8_t val) { return ((val / 16 * 10) + (val % 16)); }

time_t getRtcTime() {
  selectI2CChannel(I2C_CH_RTC);
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0)
    return 0; // Hardware fail check

  Wire.requestFrom(0x68, 7);
  if (Wire.available() < 7)
    return 0;

  tmElements_t tm;
  tm.Second = bcdToDec(Wire.read() & 0x7F);
  tm.Minute = bcdToDec(Wire.read());
  tm.Hour = bcdToDec(Wire.read() & 0x3F);
  Wire.read(); // Skip Day of Week (0x03)
  tm.Day = bcdToDec(Wire.read());
  tm.Month = bcdToDec(Wire.read());
  // Convert 2-digit year to Y2K format for TimeLib
  tm.Year = CalendarYrToTm(bcdToDec(Wire.read()) + 2000);

  return makeTime(tm);
}

void updateRTC() { // see if we need to update the RTC then update timeLib and
  bool minutesChanged = (setMinutes != px[SET_MINUTESp]);
  bool dataChanged = (setHours != px[SET_HOURSp] || setDay != px[SET_DAYp] ||
                      setMonth != px[SET_MONTHp] || setYear != px[SET_YEARp]);

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
    writeFullRtcFromSysTime(); // then update the RTC
  }
}

void writeFullRtcFromSysTime() { // load timeLib data into RTC
  selectI2CChannel(I2C_CH_RTC);
  Wire.beginTransmission(0x68);
  Wire.write(0x00);        // start at seconds
  Wire.write(decToBcd(0)); // seconds = 0
  Wire.write(decToBcd((uint8_t)minute()));
  Wire.write(decToBcd((uint8_t)hour()));
  Wire.write(0); // day of week (optional)
  Wire.write(decToBcd((uint8_t)day()));
  Wire.write(decToBcd((uint8_t)month()));
  Wire.write(decToBcd((uint8_t)year() - 2000)); // 2-digit year
  Wire.endTransmission();
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
