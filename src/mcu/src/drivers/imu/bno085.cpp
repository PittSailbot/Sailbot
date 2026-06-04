// Driver for Adafruit BNO085 9-DOF Absolute Orientation IMU
#include "drivers/imu/bno085.h"

#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>

#include "elapsedMillis.h"

static elapsedMillis last_warn;

// https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/arduino
BNO085_IMU::BNO085_IMU() : bno08x() {
}

bool BNO085_IMU::begin() {
  if (!bno08x.begin_I2C(BNO085_I2C_ADDR, &Wire)) {
    Serial.println("E: Failed to find BNO085 IMU... Check your wiring or I2C ADDR!");
    return false;
  }

  // Enable ARVR Stabilized Rotation Vector (better for orientation)
  // 50Hz report interval (20000us)
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 20000)) {
    Serial.println("E: Could not enable rotation vector report");
    return false;
  }

  this->initialized = true;
  return true;
}

bool BNO085_IMU::read(IMU* imu) {
  if (!this->initialized) {
    Serial.println("W: Trying to read from uninitialized BNO085 IMU");
    return false;
  }

  if (bno08x.wasReset()) {
    Serial.println("W: BNO085 was reset, re-enabling report");
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 20000);
  }

  // Read data
  if (!bno08x.getSensorEvent(&sensorValue)) {
    // No new data available
    return false;
  }

  if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
    uint8_t accuracy = sensorValue.status;
    if (accuracy < 2 && last_warn > 10000) {
      last_warn = 0;
      Serial.println("W: IMU not fully calibrated, move in a figure-8 slowly");
    }

    // don't write out potentially bad data
    // if (accuracy < 2) {
    //   return false; 
    // }

    // Convert quaternions to Euler angles
    double qr = sensorValue.un.arvrStabilizedRV.real;
    double qi = sensorValue.un.arvrStabilizedRV.i;
    double qj = sensorValue.un.arvrStabilizedRV.j;
    double qk = sensorValue.un.arvrStabilizedRV.k;

    double sqr = qr * qr;
    double sqi = qi * qi;
    double sqj = qj * qj;
    double sqk = qk * qk;

    // Yaw, Pitch, Roll
    // Note: Adafruit's quat_to_euler might differ slightly based on axes layout
    double yaw   = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    double pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    double roll  = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    // Convert to degrees
    yaw   = yaw * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    roll  = roll * 180.0 / M_PI;

    // Apply offsets to account for IMU mounting orientation
    imu->yaw   = fmod(yaw + this->yaw_offset + 360.0, 360.0);
    imu->pitch = fmod(pitch + this->pitch_offset + 360.0, 360.0);
    imu->roll  = fmod(roll + this->roll_offset + 360.0, 360.0);

    return true;
  }

  return false;
}
