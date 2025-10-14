#pragma once

#include "teensy.pb.h"

/**
 * @brief Common interface for all IMU implementations
 *
 * This abstract base class defines the minimal interface that all IMU drivers
 * must implement. It provides a consistent API for initialization and data reading
 * regardless of the underlying hardware.
 */
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
