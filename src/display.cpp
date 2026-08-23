#include "display.h"
#include "main.h"
//#include "debug.h"
//#include "sensors.h"
#include "utils.h"

//

                              // Create the u8x8 display                //
  // same as the NONAME variant, but uses updated SH1106 init sequence
U8X8_SH1106_128X64_WINSTAR_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);
// *****************one of these shoulf work for new display *************************
//U8X8_SSD1327_EA_W128128_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);   //try this
//U8X8_SSD1327_WS_128X128_4W_HW_SPI u8x8(/* cs=*/OLED_CS, /* dc=*/OLED_DC, /* reset=*/OLED_RESET);   /or this if text is shifted
//u8x8.setContrast(value);   //screen dimming, it doesn't belong here, it accepts 0-255
// u8x8.sendF("ca", 0x81, 0x00);  //deeper dimming last input accepts 0x00 to 0x0F

//update the display for the diffrent modes
void displayUpdate(void) {
  unsigned long mins = millis() / 60000UL;  //*********output uptime in minutes ***********
  switch (displayMode) {
    case RUN:
      dynamicInterval = DISPLAY_RATE;
      if (screenX > 5) {
        screenX = 0;
        screenY = screenY + 1;
      }
      if (screenY > 3) {
        screenX = 0;
        screenY = 0;
      }


      u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.setCursor(screenX, screenY);
      //digitalClockDisplay();
      u8x8.setCursor(screenX, screenY + 1);
      u8x8.print(sensor[0].temperature, 1);
      u8x8.print(F("F "));
      u8x8.print(sensor[0].humidity, 1);
      u8x8.print(F("%"));

      u8x8.setCursor(screenX, screenY + 2);
      u8x8.print(sensor[1].temperature, 1);
      u8x8.print(F("F "));
      u8x8.print(sensor[1].humidity, 1);
      u8x8.print(F("%"));

      u8x8.setCursor(screenX, screenY + 3);
      u8x8.print(sensor[2].temperature, 1);
      u8x8.print(F("F "));
      u8x8.print(sensor[2].humidity, 1);
      u8x8.print(F("%"));

      u8x8.setCursor(screenX, screenY + 4);
      u8x8.print(sensor[3].temperature, 1);
      u8x8.print(F("F "));
      u8x8.print(sensor[3].humidity, 1);
      u8x8.print(F("%"));
      u8x8.setCursor(screenX, screenY + 5);
      digitalClockDisplay(1);


      /*          THE OLD CODE, MOVING OUT OF THE WAY FOR NOW 
      u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.setCursor(screenX, screenY);
      //u8x8.print(mins);  //******for testing purposes.
      digitalClockDisplay(1);
      u8x8.setCursor(screenX, screenY + 1);
      u8x8.print(temperature, 1);
      u8x8.print(F("F"));
      u8x8.setCursor(screenX, screenY + 2);
      u8x8.print(humidity, 1);
      u8x8.print(F("%"));
      u8x8.setCursor(screenX, screenY + 3);
      u8x8.print(NTCtempHeatblock, 1);
      u8x8.print(F("F"));
      u8x8.setCursor(screenX, screenY + 4);
      u8x8.print(NTCtempHeatsink, 1);
      u8x8.print(F("F"));
      /*if (modeState == FRUIT) {
        u8x8.print(F("FRUIT"));
      }
      if (modeState == INCUBATE) {
        u8x8.print(F("INCUBA"));
      }
      u8x8.setCursor(screenX, screenY + 5);
      u8x8.print(freeRam());
      */
      screenX = screenX + 1;


      break;
    case MENU:
      dynamicInterval = DISPLAY_PROGRAM_RATE;
      runMenuCode = true;
      u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.setCursor(5, 0);
      u8x8.print(F("MENU"));
      if (menuSelector == 1) {
        u8x8.setCursor(1, 2);
        u8x8.print(F(">>>FRUIT<<<"));
      } else {
        u8x8.setCursor(4, 2);
        u8x8.print(F("FRUIT"));
      }
      if (menuSelector == 2) {
        u8x8.setCursor(1, 3);
        u8x8.print(F(">>>INCUBATE<<<"));
      } else {
        u8x8.setCursor(4, 3);
        u8x8.print(F("INCUBATE"));
      }
      if (menuSelector == 3) {
        u8x8.setCursor(1, 4);
        u8x8.print(F(">>>STATUS<<<"));
      } else {
        u8x8.setCursor(4, 4);
        u8x8.print(F("STATUS"));
      }

      if (menuSelector == 4) {
        u8x8.setCursor(1, 5);
        u8x8.print(F(">>>PROGRAM<<<"));
      } else {
        u8x8.setCursor(4, 5);
        u8x8.print(F("PROGRAM"));
      }
      break;
    case STATUS:
      dynamicInterval = DISPLAY_PROGRAM_RATE;
      //u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.setCursor(0, 0);  //display time and mode
      u8x8.print(freeRam());
      u8x8.print(F(" RAM"));
      u8x8.setCursor(8, 0);
      if (modeState == FRUIT) {
        u8x8.print(F("   FRUIT"));
      }
      if (modeState == INCUBATE) {
        u8x8.print(F("INCUBATE"));
      }
      u8x8.setCursor(0, 1);  //display temp status
      u8x8.print(temperature, 1);
      u8x8.print(F("F"));
      u8x8.setCursor(7, 1);
      u8x8.print(F("SET="));
      u8x8.print(setpoint, 1);
      u8x8.print(F("F"));

      u8x8.setCursor(0, 2);  //display humidity status
      u8x8.print(humidity, 1);
      u8x8.print(F("%"));
      u8x8.setCursor(7, 2);
      u8x8.print(F("SET="));
      u8x8.print(humiditySetpoint, 1);
      u8x8.print(F("%"));

      u8x8.setCursor(0, 3);  //display heat spreader temp
      u8x8.print(F("BLOCK TEMP "));
      u8x8.print(NTCtempHeatblock, 1);
      u8x8.print(F("F"));


      xShift = rightJustify(pwmDrive);
      u8x8.setCursor(0, 4);
      if (inHeatMode == true) {
        u8x8.print(F("HEAT ON       "));
        u8x8.setCursor(14 - xShift, 4);
        u8x8.print(pwmDrive, 0);
        u8x8.setCursor(15, 4);
        u8x8.print(F("%"));
      } else {

        u8x8.print(F("COOL ON       "));
        u8x8.setCursor(14 - xShift, 4);
        u8x8.print(pwmDrive, 0);
        u8x8.setCursor(15, 4);
        u8x8.print(F("%"));
      }
      u8x8.setInverseFont(0);

      //x = px[LIGHT_LEVELp] * 100 / 7;
      xShift = rightJustify(px[LIGHT_LEVELp] * 100 / 7);
      u8x8.setCursor(0, 5);
      if (digitalRead(LIGHT_PIN1) || digitalRead(LIGHT_PIN2) || digitalRead(LIGHT_PIN3) || digitalRead(LIGHT_PIN4)) {
        u8x8.setInverseFont(1);
        u8x8.print(F("LIGHTS  SET=   "));
        u8x8.setCursor(14 - xShift, 5);
        u8x8.print(px[LIGHT_LEVELp] * 100 / 7, 0);
        u8x8.setCursor(15, 5);
        u8x8.print(F("%"));
      } else {
        u8x8.setInverseFont(0);
        u8x8.print(F("LIGHTS  SET=   "));
        u8x8.setCursor(14 - xShift, 5);
        u8x8.print(px[LIGHT_LEVELp] * 100 / 7, 0);
        u8x8.setCursor(15, 5);
        u8x8.print(F("%"));
      }

      u8x8.setInverseFont(0);
      if (digitalRead(AIRPUMP_PIN)) {
        u8x8.setInverseFont(1);
      }
      xShift = rightJustify(px[AIR_SECONDSp]);
      u8x8.setCursor(0, 6);  //display airpump status
      u8x8.print(F("AIR SEC        "));
      u8x8.setCursor(15 - xShift, 6);
      u8x8.print(px[AIR_SECONDSp], 0);
      u8x8.setInverseFont(0);

      if (digitalRead(HUMIDIFIER_PIN)) {
        u8x8.setInverseFont(1);
      }
      xShift = rightJustify(px[WATER_SECONDSp]);
      u8x8.setCursor(0, 7);  //display huumidifier status
      u8x8.print(F("WATER SEC      "));
      u8x8.setCursor(15 - xShift, 7);
      u8x8.print(px[WATER_SECONDSp], 0);
      u8x8.setInverseFont(0);
      break;


    case PROGRAM:
      dynamicInterval = DISPLAY_PROGRAM_RATE;
      runProgramCode = true;
      displayProgram();
      break;
  }
}

//display handler for pprogramming menu 
void displayProgram(void) {
  //program by default so we will used the lower part of the screen from program mode.
  double X = px[currentParam];  //the following is to
  int shiftPlaces = 8;          //align the decimal point of all parameters in display
  if (X < 1) {
    shiftPlaces = 7;
  } else {
    for (; X >= 1; shiftPlaces--) {
      X = X * .1;
    }
  }
  //u8x8.setTextSize(1);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  //u8x8.print(parameterNames[currentParam]);  //really bad name? lol  Print the name of the parameter
  char* currentStringPointer = (char*)pgm_read_word(&(parameterNames[currentParam]));
  u8x8.print(currentStringPointer);
  //DEBUG_PRINTLN(currentStringPointer);
  u8x8.setCursor(1, 3);  //location of parameter data
  if (currentParam < EXIT) {
    while (shiftPlaces-- > 0)         //loop the number of time
      u8x8.print(F(" "));             //spaces for deciaml alignemnt
    u8x8.print(px[currentParam], 4);  //menuPointer currentParam edit
  } else {
    u8x8.print(F("   "));
    u8x8.print(menuOptions[menuYes]);
  }

  u8x8.setCursor(0, 1);
  if (edit && (menuPointer == 10)) {  //handle the pointer for parameter change selection
    u8x8.print(F("^^^^^^^^^^^"));
  } else if (!edit && (menuPointer == 10)) {
    u8x8.print(F("***********"));  //now in move the pointer mode
  } else {
    //u8x8.print(F(" "));
  }
  if (menuPointer != 10) {  //are we editing parameter values?
    u8x8.setCursor(1, 4);   //move cursor below target data value
    for (int i = 10; i > -1; i--) {
      if (i == 2) {
        u8x8.print(F(" "));
      }
      if ((i - menuPointer) == 0) {
        if (edit) {
          u8x8.print(F("^"));  //plave edit cursor ^ under proper digit.
        } else if (!edit) {    //place select cursor * under proper digit
          u8x8.print(F("*"));
        }
      } else {
        u8x8.print(F(" "));
      }
    }
  }


  u8x8.setCursor(0, 7);  //show what parameter # is being edited
  u8x8.print(F("P#"));
  u8x8.print(currentParam);  //mostly for programming debug, but nice.
                             //DEBUG_PRINTLN(freeRam());
}

//retuurns the number of digit shifts to right justify numerical vars
int rightJustify(double value) {
  if (value >= 100) {
    return 2;
  } else if (value >= 10) {
    return 1;
  } else {
    return 0;
  }
}

//output time to OLED
void digitalClockDisplay(uint8_t mode) {  // digital clock display of the time
  switch (mode) {
    case 1:
      {

        int hours = hour();
        bool AM = true;
        if (hours >= 12) {
          AM = false;
        }
        if (hours > 12) {
          hours = hours - 12;
        }
        if (hours == 0) {
          hours = 12;
        }
        u8x8.print(hours);
        u8x8.print(F(":"));
        printDigits(minute());
        if (AM == true) {
          u8x8.print(F("AM"));
        } else {
          u8x8.print(F("PM"));
        }
        //printDigits(second());
        break;
      }
    case 2:
      {
        char* currentStringPointer = (char*)pgm_read_word(&(monthNames[month() - 1]));
        u8x8.print(currentStringPointer);
        u8x8.print(F(" "));
        u8x8.print(day());
        break;
      }
    case 3:
      {
        u8x8.print(year());
        break;
      }
  }
}


// utility function for digital clock display: prints leading 0
void printDigits(int digits) {
  if (digits < 10)
    u8x8.print(F("0"));
  u8x8.print(digits);
}

