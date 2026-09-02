// =============== PELTIER HALT MODE ===============
#define HALT_MODE 0  //  1 disables the peltier and fans for development.

#if HALT_MODE
#define SET_PWM(dutyCycle) (OCR2A = 0)  // Peltier forced OFF
#define SET_FAN(fanSpeed) (OCR1A = 0)   // Peltier forced OFF
#else
#define SET_PWM(dutyCycle) (OCR2A = dutyCycle)                          // Normal inverted PWM
#define SET_FAN(fanSpeed) (OCR1A = (uint8_t)(fanSpeed * 2.55f + 0.5f))  // Peltier forced OFF
#endif

//==================  TURN ON DEBUGGING OUTPUT ===============
#define DEBUG_MODE 1  // Set to 0 to recover ~200 bytes of RAM

#if DEBUG_MODE
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_BEGIN(x) Serial.begin(x)
#define DEBUG_FLUSH() Serial.flush()
#else
#define DEBUG_PRINT(...)    // Does nothing
#define DEBUG_PRINTLN(...)  // Does nothing
#define DEBUG_BEGIN(x)      // Does nothing
#define DEBUG_FLUSH()
#endif
