/*
   PascalQuad Sensor Testbed

   These values will effect the timing:
   CPU Speed: 180MHz
   Optimize: Fastest

  sensors will update at the beginning of each loop iff they have data
    - i2c interrupts are checked -> if high, req new data (verify rate and time taken)
    - gps serial buffer is checked -> if something there, everything read (verify rate and time taken)
  every loop, imu quanternion filter will update
  every loop, kalman filter/system model will update
  every loop, updates will be sent to motors from controller
  keeping these last three tasks completing as fast as possibile is vital
    - ensures gps serial data is not kept waiting long
    - time the update parts of the loop and determine system update rate
    - determine how much of an impact sensor updates are (percentage of time reading sensors vs updating system)
  system update rate will be faster than sensor update rate
    - different than last quadcopter
    - will make the system model and kalman filter very important to get right
  sensor update rates will be kept reasonable, not super fast
  serial transmission offboard will be kept minimal
    - not every loop, but at predetermined rate (like 2Hz or something)


    TODO
    - sd card recording - time how long reads and writes take
    - set up bluetooth
    - time an empty loop to verify processor speed
    - write Kalman filter class

*/

#include <Arduino.h>
#include <i2c_t3.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include "SparkFunMPL3115A2.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "SevSeg.h"


#define SERIAL_DEBUG_TESTBED


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


/** SWITCHES **/
// set pin numbers:
const int SW1 = 39;
const int SW2 = 38;
const int SW3 = 37;
const int SW4 = 36;
int SW_array = 0;


/** SEVSEG **/
SevSeg sevseg; //Instantiate a seven segment controller object
byte numDigits = 3;
byte digitPins[] = {33, 34, 35};
byte segmentPins[] = {18, 19, 22, 21, 20, 17, 16, 23};
bool resistorsOnSegments = false;    // 'false' means resistors are on segment pins
byte hardwareConfig = N_TRANSISTORS; // See README.md for options
bool updateWithDelays = false;       // Default 'false' is Recommended
bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected
char startMessage[4] = "HEY";


/** LEDS **/
const int LED = 13;
bool LED_STATE = LOW;


/** TIMING **/
elapsedMillis pressure_timer = 0;
elapsedMillis display_timer = 0;
elapsedMicros micros_timer = 0;
elapsedMillis millis_timer = 0;
long timer_result = -1;
time_t prevDisplay = 0; // when the digital clock was displayed