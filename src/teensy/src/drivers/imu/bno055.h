#pragma once

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#include "imu.h"

#define BNO055_I2C_ADDR 0x28  // 0x28 or 0x29 (if soldered)

/**
 * @brief Driver for Adafruit BNO055 9-DOF Absolute Orientation IMU
 *
 * The BNO055 is an all-in-one sensor that provides absolute orientation
 * with built-in sensor fusion, requiring minimal configuration.
 *
 * @see https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 */
class BNO055_IMU : public IMUInterface {
 private:
  Adafruit_BNO055 bno;

 public:
  BNO055_IMU();
  // ~BNO055_IMU();
  bool begin() override;
  bool read(IMU* imu) override;
};
