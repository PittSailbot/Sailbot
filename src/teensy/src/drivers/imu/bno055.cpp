// Driver for Adafruit BNO055 9-DOF Absolute Orientation IMU
#include "drivers/imu/bno055.h"

#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
BNO055_IMU::BNO055_IMU() : bno(55, BNO055_I2C_ADDR, &Wire) {
}

bool BNO055_IMU::begin() {
  if (!bno.begin()) {
    Serial.println("E: Failed to find BNO055 IMU... Check your wiring or I2C ADDR!");
    return false;
  }

  // bno.setExtCrystalUse(true);

  this->initialized = true;
  return true;
}

bool BNO055_IMU::read(IMU* imu) {
  if (!this->initialized) {
    return false;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  imu->yaw = event.orientation.x;
  imu->pitch = event.orientation.y;
  imu->roll = event.orientation.z;
  // Acceleration can be collected but unused
  // imu->accel_x = event.acceleration.x

  return true;
}
