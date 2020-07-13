#include "header.h"


void setup() {

  /** SERIAL **/
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Type any character to start..."));
  while (!Serial.available());

  Serial.println(F("Beginning sketch!"));
#endif

  /** SWITCHES **/
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  /** SEVSEG **/
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
               updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(0); // range is -200 to 200

  /** LEDS **/
  pinMode(LED, OUTPUT);

  /** I2C **/
  Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, I2C_CLOCK); // Wire bus, SCL pin 7, SDA pin 8, ext pullup, 400kHz

  /** IMU **/
  myIMU.MPU9250SelfTest(myIMU.selfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();

  /** ALTIMETER **/
  myMPL.begin(Wire2, MPL3115A2_ADDRESS);    // get sensor online
  myMPL.setModeAltimeter(); // Measure altitude above sea level in meters
  myMPL.setOversampleRate(7); // Set Oversample to the recommended 128
  myMPL.enableEventFlags(); // Enable all three pressure and temp event flags

  /** GPS **/
  gpsPort.begin(115200);

  /** SERIAL **/
#ifdef SERIAL_DEBUG
  Serial.println(F("Finished Setup!"));
  Serial.println(F("Data will be sent to Serial when GPS connection is established."));
  Serial.println(F("Time/Date\tAltitude\tYaw\tPitch\tRoll\tLatitude\tLongitude\tLocAge\tSatellites"));
#endif

}



void loop() {

  /** IMU **/
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) { // takes 615us, runs at about 200Hz (every 5ms)
    myIMU.readAccelData(myIMU.accelCount);  // takes 247us
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
    myIMU.readGyroData(myIMU.gyroCount);  // takes 247us
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
    myIMU.readMagData(myIMU.magCount);  // takes 121us
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }


  /** IMU FILTER **/ // (RUNS EVERY LOOP, takes 83us at most, runs at 5000Hz (every 200us), with some gaps when sensor data taken)
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                              * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                      * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                      * *(getQ() + 3));
  myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                              * *(getQ() + 2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                              * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                      * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                      * *(getQ() + 3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;

  // Current Location: Research Park, Huntsville, Alabama
  // Current Declination: +3.783333 ±0.3666667 degrees (changing +0.06666667 degrees per year)
  myIMU.yaw  += 3.783333;
  myIMU.roll *= RAD_TO_DEG;


  /** ALTIMETER **/
  if (pressure_timer > 512) { // every 512ms (corresponds with setting of oversample rate)
    pressure_timer = 0;

    // command will force a sample to be taken. update rate is still important
    myMPL.readAltitude(); // 696us at 400kHz i2c rate
  }


  /** GPS **/
  // must run repeatedly. should only take 3us at most, but must run at at least 1Hz
  while (gpsPort.available()) {
    if (gps.encode(gpsPort.read())) {
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      if (age < 500) {
        // set the Time to the latest GPS reading
        setTime(Hour, Minute, Second, Day, Month, Year);
        adjustTime(offset * SECS_PER_HOUR);
      }
    }
  }


  /** SWITCHES **/
  bitWrite(SW_array, 0, !digitalRead(SW1));
  bitWrite(SW_array, 1, !digitalRead(SW2));
  bitWrite(SW_array, 2, !digitalRead(SW3));
  bitWrite(SW_array, 3, !digitalRead(SW4));

  if (display_timer > 100) {
    switch (SW_array) {
      case 0:
        sevseg.setChars(startMessage);
        break;
      case 1:
        sevseg.setNumber(round(myMPL.altitude));
        break;
      case 2:
        sevseg.setNumber(myMPL.temperature, 1);
        break;
      case 3:
        sevseg.setNumber(gps.satellites());
        break;
      case 4:
        gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
        sevseg.setNumber(age);
        break;
      case 5:
        sevseg.setNumber(timer_result);
        break;
      case 6:
        sevseg.setNumber(myIMU.yaw, 1);
        break;
      case 7:
        sevseg.setNumber(myIMU.pitch, 1);
        break;
      case 8:
        sevseg.setNumber(myIMU.roll, 1);
        break;
      case 9:
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
      default:
        sevseg.setNumber(SW_array, 0);
        break;
    }

    display_timer = 0;
  }


  /** SEVSEG **/
  sevseg.refreshDisplay(); // Must run every loop, takes 1-2 micros


  /** SERIAL **/
#ifdef SERIAL_DEBUG
  String dataString = "";
  dataString += String(year());
  dataString += String(month());
  dataString += String(day());
  dataString += " ";
  if (hour() < 10) dataString += "0";
  dataString += String(hour());
  dataString += ":";
  if (minute() < 10) dataString += "0";
  dataString += String(minute());
  dataString += ":";
  if (second() < 10) dataString += "0";
  dataString += String(second());
  dataString += "\t";
  dataString += String(myMPL.altitude);
  dataString += "\t";
  dataString += String(myIMU.yaw);
  dataString += "\t";
  dataString += String(myIMU.pitch);
  dataString += "\t";
  dataString += String(myIMU.roll);
  dataString += "\t";
  dataString += String(flat);
  dataString += "\t";
  dataString += String(flon);
  dataString += "\t";
  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
  dataString += String(age);
  dataString += "\t";
  dataString += String(gps.satellites());

  if (now() != prevDisplay) { //update the display only if the time has changed
    prevDisplay = now();
    Serial.println(dataString);
  }
#endif

}