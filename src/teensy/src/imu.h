#ifndef IMU_H
#define IMU_H

#include "teensy.pb.h"

#define FILTER_UPDATE_RATE_HZ 100

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
  virtual bool setup() = 0;

  /**
   * @brief Update IMU sensors filters
   */
  virtual void update() = 0;

  /**
   * @brief Read current IMU data
   * @param imu Pointer to IMU protobuf to populate with data
   * @return true if data was successfully read, false otherwise
   */
  virtual bool read(IMU* imu) = 0;
};

int setupIMU();

bool readIMU(IMU* imu);

#endif