#pragma once

#include <Adafruit_BNO08x.h>

#include "drivers/imu/imu.h"

#define BNO085_I2C_ADDR 0x4A  // 0x4A or 0x4B (if soldered)

/**
 * @brief Driver for Adafruit BNO085 9-DOF Absolute Orientation IMU
 *
 * The BNO085 is an all-in-one sensor that provides absolute orientation
 * with built-in sensor fusion, requiring minimal configuration.
 *
 * @see https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085
 */
class BNO085_IMU : public IMUInterface {
 private:
  Adafruit_BNO08x bno08x;
  sh2_SensorValue_t sensorValue;

 public:
  BNO085_IMU();
  // ~BNO085_IMU();
  bool begin() override;
  bool read(IMU* imu) override;
};
