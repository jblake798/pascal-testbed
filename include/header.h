#include <Arduino.h>
#include <i2c_t3.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include <SPI.h>
#include "SparkFunMPL3115A2.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "SevSeg.h"
#include "SdFat.h"

// this process takes 250us, currently (dependent on size of dataString)
#define SERIAL_DEBUG_PASCAL

// this process takes up to 16ms. Sometimes double this time for two iterations.
// only use if timing not crucial or if logging sporadically.
// #define SD_DATALOG_PASCAL


/** I2C **/
#define I2C_CLOCK 400000


/** IMU **/
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define IMU_FILTER_TYPE MADGWICK_FILTER

MPU9250 myIMU(MPU9250_ADDRESS, Wire2, I2C_CLOCK);


/** ALTIMETER **/
MPL3115A2 myMPL;


/** GPS **/
#define gpsPort Serial1
#define _GPS_NO_STATS

TinyGPS gps;
float flat, flon;
float falt_gps;
int Year;
byte Month, Day, Hour, Minute, Second;
unsigned long age = 0;


/** TIME **/
const int offset = -5;  // Eastern Standard Time (USA)
//const int offset = -4;  // Eastern Daylight Time (USA)


/** SD CARD **/
#ifdef SD_DATALOG_PASCAL

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t chipSelect = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t chipSelect = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS

SdFs sd;
FsFile file;
char fileName[20] = {'\0'};
time_t currentFileStart = 0;

#define error(s) sd.errorHalt(&Serial, F(s))

#endif  // SD_DATALOG_PASCAL


/** SWITCHES **/
// set pin numbers:
const int SW1 = 15;
const int SW2 = 14;
const int SW3 = 39;
const int SW4 = 38;
int SW_array = 0;


/** SEVSEG **/
SevSeg sevseg; //Instantiate a seven segment controller object
byte numDigits = 5;
byte digitPins[] = {33, 34, 35, 36, 37};
byte segmentPins[] = {18, 19, 22, 21, 20, 17, 16, 23};
bool resistorsOnSegments = false;    // 'false' means resistors are on segment pins
byte hardwareConfig = N_TRANSISTORS; // See README.md for options
bool updateWithDelays = false;       // Default 'false' is Recommended
bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected
char startMessage[6] = "PASCL";


/** LEDS **/
const int LED = 13;
bool LED_STATE = LOW;


/** TIMING **/
elapsedMillis pressure_timer = 0;
elapsedMillis display_timer = 0;
elapsedMicros micros_timer = 0;
elapsedMillis millis_timer = 0;
long timer_result = -1;
time_t prevSDRecord = 0; // when the SD Card was last updated
time_t prevSerialDisplay = 0; // when the Serial was last updated