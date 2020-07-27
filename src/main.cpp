#include "header.h"


void setup() {

  /** SERIAL **/
#ifdef SERIAL_DEBUG_PASCAL
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Type any character to start..."));
  while (!Serial.available());

  Serial.println(F("Beginning sketch!"));
#endif  // SERIAL_DEBUG_PASCAL

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
  IMUstatus = myIMU.begin();
  if (IMUstatus < 0) {
#ifdef SERIAL_DEBUG_PASCAL
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(IMUstatus);
#endif  // SERIAL_DEBUG_PASCAL

  } else {
    // setting the accelerometer full scale range to +/-16G 
    myIMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
    // setting the gyroscope full scale range to +/-2000 deg/s
    myIMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
    // setting DLPF bandwidth to 41 Hz
    myIMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    // setting SRD to 9 for a 100 Hz update rate
    myIMU.setSrd(9);
    // set Accelerometer biases and scaling
    myIMU.setAccelCalX(0.2522, 1.0022);
    myIMU.setAccelCalY(0.0933, 1.0008);
    myIMU.setAccelCalZ(0.1208, 0.9803);
    // set Magnetometer biases and scaling (TODO: refine with MPU9250Calibration.ino util when battery pack arrives)
    // myIMU.setMagCalX(28.55860519, 1.19339991);
    // myIMU.setMagCalY(31.16550255, 1.51063430);
    // myIMU.setMagCalZ(20.59740067, 0.66662914);
    // set up IMU interrupts
    myIMU.enableDataReadyInterrupt(); // can attach interrupt pin to processor interrupt routine

  }

  /** ALTIMETER **/
  myMPL.begin(Wire2, MPL3115A2_ADDRESS);    // get sensor online
  myMPL.setModeAltimeter(); // Measure altitude above sea level in meters
  myMPL.setOversampleRate(7); // Set Oversample to the recommended 128
  myMPL.enableEventFlags(); // Enable all three pressure and temp event flags

  /** GPS **/
  gpsPort.begin(115200);

  /** SD CARD **/
#ifdef SD_DATALOG_PASCAL
  if (!sd.begin(SD_CONFIG)) {
    LED_STATE = HIGH;
    digitalWrite(LED, LED_STATE); // 1 us
#ifdef SERIAL_DEBUG_PASCAL
    sd.initErrorHalt(&Serial);
#endif  // SERIAL_DEBUG_PASCAL
  } else {
    LED_STATE = LOW;
    digitalWrite(LED, LED_STATE); // 1 us
  }
#endif  // SD_DATALOG_PASCAL

  /** SERIAL **/
#ifdef SERIAL_DEBUG_PASCAL
  Serial.println(F("Finished Setup!"));
  Serial.println(F("Data will be sent to Serial when GPS connection is established."));
  Serial.println(F("Date\t\tTime\t\tAlt\tYaw\tPitch\tRoll\tLat\tLong\tAge\tSats"));
#endif  // SERIAL_DEBUG_PASCAL

}



void loop() {

  /** IMU **/ // takes 622us. can also be triggered by interrupt.
  if (myIMU.checkInterrupt()) {
    myIMU.readSensor();
  }


  // /** IMU FILTER **/ // (RUNS EVERY LOOP)



  /** ALTIMETER **/
  if (pressure_timer > 512) { // every 512ms (corresponds with setting of oversample rate)
    pressure_timer = 0;

    // command will force a sample to be taken. update rate is still important
    myMPL.readAltitude(); // takes 696us at 400kHz i2c rate
  }


  /** GPS **/
  // must run repeatedly. should only take 3us at most, but must run at at least 1Hz
  while (gpsPort.available()) {
    if (gps.encode(gpsPort.read())) {
      gps.f_get_position(&flat, &flon, &age);
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
        sevseg.setNumber(myIMU.getAccelX_mss(), 1);
        break;
      case 7:
        sevseg.setNumber(myIMU.getAccelY_mss(), 1);
        break;
      case 8:
        sevseg.setNumber(myIMU.getAccelZ_mss(), 1);
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
#ifdef SERIAL_DEBUG_PASCAL
  // takes 250-280us, depending on size of variables (highly dependent on this)
  if (now() != prevSerialDisplay) { //update the display only every second
    String dataString = "";
    dataString += String(year());
    if (month() < 10) dataString += "0";
    dataString += String(month());
    if (day() < 10) dataString += "0";
    dataString += String(day());
    dataString += "\t";
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
    dataString += String(myIMU.getAccelX_mss());
    dataString += "\t";
    dataString += String(myIMU.getAccelY_mss());
    dataString += "\t";
    dataString += String(myIMU.getAccelZ_mss());
    dataString += "\t";
    dataString += String(flat);
    dataString += "\t";
    dataString += String(flon);
    dataString += "\t";
    gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
    dataString += String(age);
    dataString += "\t";
    dataString += String(gps.satellites());

    prevSerialDisplay = now();
    Serial.println(dataString);
  }
#endif  // SERIAL_DEBUG_PASCAL


  /** SD CARD **/
#ifdef SD_DATALOG_PASCAL
  if ((timeStatus() != timeNotSet) && (year() >= 2020)) {   // log data only if gps connection has been established / time is set
    if (now() != prevSDRecord) {   // log data only if the time has changed (every second)
      // this process takes up to 16ms. Sometimes double this time for two iterations.
      // only use if timing not crucial or if logging sporadically.

      prevSDRecord = now();

      if ((currentFileStart == 0) || (hour() != hour(currentFileStart))) {
        currentFileStart = now();

        if (file.isOpen()) {
          file.close();
        }

        sprintf(fileName, "%4d%02d%02d_%02d00.CSV", year(currentFileStart), month(currentFileStart), day(currentFileStart),
                hour(currentFileStart));

        if (sd.exists(fileName)) {
          if (!file.open(fileName, FILE_WRITE)) {
#ifdef SERIAL_DEBUG_PASCAL
          error("open failed");
#endif  // SERIAL_DEBUG_PASCAL

          }

        } else {
          if (!file.open(fileName, FILE_WRITE)) {
#ifdef SERIAL_DEBUG_PASCAL
            error("open failed");
#endif  // SERIAL_DEBUG_PASCAL

          } else {
            file.print(F("Date,Time,Altitude,Yaw,Pitch,Roll,Latitude,Longitude,LocAge,Satellites\r\n"));

          }
        }
      }

      String dataString = "";
      dataString += String(year());
      if (month() < 10) dataString += "0";
      dataString += String(month());
      if (day() < 10) dataString += "0";
      dataString += String(day());
      dataString += ",";
      if (hour() < 10) dataString += "0";
      dataString += String(hour());
      dataString += ":";
      if (minute() < 10) dataString += "0";
      dataString += String(minute());
      dataString += ":";
      if (second() < 10) dataString += "0";
      dataString += String(second());
      dataString += ",";
      dataString += String(myMPL.altitude);
      dataString += ",";
      dataString += String(myIMU.yaw);
      dataString += ",";
      dataString += String(myIMU.pitch);
      dataString += ",";
      dataString += String(myIMU.roll);
      dataString += ",";
      dataString += String(flat);
      dataString += ",";
      dataString += String(flon);
      dataString += ",";
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      dataString += String(age);
      dataString += ",";
      dataString += String(gps.satellites());
      dataString += "\r\n";

      file.seekEnd();
      file.print(dataString); // takes 1.2ms, sometimes double

      if (!file.sync() || file.getWriteError()) { // takes 14.5ms
#ifdef SERIAL_DEBUG_PASCAL
        error("write error");
#endif  // SERIAL_DEBUG_PASCAL
      }
    }
  }
#endif  // SD_DATALOG_PASCAL

}