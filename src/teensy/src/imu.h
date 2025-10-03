#pragma once

#include <Adafruit_AHRS.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor_Calibration.h>

#include "teensy.pb.h"

#define BNO055_I2C_ADDR 0x28  // 0x28 or 0x29 (if soldered)
#define LSM6DS_I2C_ADDR 0x6A  // 0x6A or 0x6B (if soldered)
class IMUInterface {
 public:
  bool initialized = false;
  // TODO: offset difference between boat's straight and IMU's straight, depends on mounting
  // int offset_yaw

  virtual ~IMUInterface() = default;

  /**
   * @brief Initialize the IMU hardware
   * @return true if initialization succeeded, false otherwise
   */
  virtual bool begin() = 0;

  /**
   * @brief Read current IMU data
   * @param imu Pointer to IMU protobuf to populate with data
   * @return true if data was successfully read, false otherwise
   */
  virtual bool read(IMU* imu) = 0;
};

class BNO055_IMU : public IMUInterface {
 private:
  Adafruit_BNO055 bno;

 public:
  BNO055_IMU();
  // ~BNO055_IMU();
  bool begin() override;
  bool read(IMU* imu) override;
};

class LSM6DS_IMU : public IMUInterface {
 private:
  Adafruit_LSM6DS3TRC lsm6ds;
  Adafruit_LIS3MDL lis3mdl;
  Adafruit_Sensor* accelerometer;
  Adafruit_Sensor* gyroscope;
  Adafruit_Sensor* magnetometer;

  // pick your filter! slower == better quality output
  Adafruit_NXPSensorFusion filter;  // slowest
  // Adafruit_Madgwick filter;  // faster than NXP
  //  Adafruit_Mahony filter;  // fastest/smalleset
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