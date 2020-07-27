#include <Arduino.h>
#include "MPU9250.h"

// #define ACCL_CALIB
// #define GYRO_CALIB
// #define MAGN_CALIB

#define INT_REPORT

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire2, 0x68);
int status;

void setup() {
    // serial to display data
    Serial.begin(115200);
    while (!Serial) {}

    // start communication with IMU
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1) {}
    }
    // setting the accelerometer full scale range to +/-16G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
    // setting the gyroscope full scale range to +/-2000 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
    // setting DLPF bandwidth to 41 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    // setting SRD to 9 for a 100 Hz update rate
    IMU.setSrd(9);

#ifdef INT_REPORT
    // enabling the data ready interrupt
    IMU.enableDataReadyInterrupt();
    // attaching the interrupt to microcontroller pin
    pinMode(5, INPUT);
    attachInterrupt(5, getIMU, RISING);
#endif //INT_REPORT

    // previously found calibration data
    IMU.setAccelCalX(0.25805569, 1.00042164);
    IMU.setAccelCalY(0.07447767, 1.00316799);
    IMU.setAccelCalZ(0.19314232, 0.97985035);

    IMU.setMagCalX(28.55860519, 1.19339991);
    IMU.setMagCalY(31.16550255, 1.51063430);
    IMU.setMagCalZ(20.59740067, 0.66662914);
}

void loop() {

#ifndef INT_REPORT

    Serial.println("Starting!");

#ifdef ACCL_CALIB
    // must be performed for each of 6 axes to get calibration data.

    // serial prints can be added inside this function to verify min/maxing is correct
    // a delay and !Serial.available could also be added to incorporate waiting for a flip of the IMU
    status = IMU.calibrateAccel();

    Serial.print(status);
    Serial.print("\t");
    Serial.print(IMU.getAccelBiasX_mss(), 8);
    Serial.print("\t");
    Serial.print(IMU.getAccelScaleFactorX(), 8);
    Serial.print("\t");
    Serial.print(IMU.getAccelBiasY_mss(), 8);
    Serial.print("\t");
    Serial.print(IMU.getAccelScaleFactorY(), 8);
    Serial.print("\t");
    Serial.print(IMU.getAccelBiasZ_mss(), 8);
    Serial.print("\t");
    Serial.println(IMU.getAccelScaleFactorZ(), 8);

#endif

#ifdef MAGN_CALIB
    // must be performed while device is moved in figure-8 motion

    // serial prints can be added inside this function to verify min/maxing is correct
    // a delay and !Serial.available could also be added to incorporate waiting for a flip of the IMU
    status = IMU.calibrateMag();

    Serial.print(status);
    Serial.print("\t");
    Serial.print(IMU.getMagBiasX_uT(), 8);
    Serial.print("\t");
    Serial.print(IMU.getMagScaleFactorX(), 8);
    Serial.print("\t");
    Serial.print(IMU.getMagBiasY_uT(), 8);
    Serial.print("\t");
    Serial.print(IMU.getMagScaleFactorY(), 8);
    Serial.print("\t");
    Serial.print(IMU.getMagBiasZ_uT(), 8);
    Serial.print("\t");
    Serial.println(IMU.getMagScaleFactorZ(), 8);

#endif //MAGN_CALIB

    Serial.println("Waiting for next run....");
    while (!Serial.available()) {
        while (!Serial.available()) {}
        Serial.clear();
        // read the sensor
        IMU.readSensor();
        // display the data
        Serial.print(IMU.getAccelX_mss(), 6);
        Serial.print("\t");
        Serial.print(IMU.getAccelY_mss(), 6);
        Serial.print("\t");
        Serial.print(IMU.getAccelZ_mss(), 6);
        Serial.print("\t");
        Serial.print(IMU.getGyroX_rads(), 6);
        Serial.print("\t");
        Serial.print(IMU.getGyroY_rads(), 6);
        Serial.print("\t");
        Serial.print(IMU.getGyroZ_rads(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagX_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagY_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagZ_uT(), 6);
        Serial.print("\t");
        Serial.println(IMU.getTemperature_C(), 6);
        delay(10);
    }
    Serial.clear();

#endif //INT_REPORT

}

void getIMU() {
    // read the sensor
    IMU.readSensor();
    // display the data
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);
    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(), 6);
}