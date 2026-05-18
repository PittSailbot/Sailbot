#pragma once

#include <Adafruit_AHRS.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Sensor_Calibration.h>

#include "drivers/imu/imu.h"

#define LSM6DS_I2C_ADDR 0x6A  // 0x6A or 0x6B (if soldered)

/**
 * @brief Driver for LSM6DS3TR-C + LIS3MDL 9-DOF IMU combo
 *
 * This driver combines the LSM6DS3TR-C (accelerometer + gyroscope) with
 * the LIS3MDL (magnetometer) to create a 9-DOF IMU with sensor fusion.
 * Requires calibration and regular updates for accurate orientation data.
 *
 * @see https://learn.adafruit.com/adafruit-lsm6ds3tr-c-lis3mdl-precision-9-dof-imu/arduino
 */
class LSM6DS_IMU : public IMUInterface {
 private:
  Adafruit_LSM6DS3TRC lsm6ds;
  Adafruit_LIS3MDL lis3mdl;
  Adafruit_Sensor* accelerometer;
  Adafruit_Sensor* gyroscope;
  Adafruit_Sensor* magnetometer;

  // Pick your filter! slower == better quality output
  Adafruit_NXPSensorFusion filter;  // slowest
  // Adafruit_Madgwick filter;  // faster than NXP
  // Adafruit_Mahony filter;  // fastest/smallest
  static constexpr int FILTER_UPDATE_RATE_HZ = 10;
  uint32_t timestamp;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

  bool init_sensors();

 public:
  LSM6DS_IMU();
  //~LSM6DS_IMU();
  bool begin() override;
  void update();
  bool read(IMU* imu) override;
};
