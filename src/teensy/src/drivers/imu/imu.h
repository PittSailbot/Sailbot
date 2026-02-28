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

  // Offsets to account for IMU mounting orientation relative to boat (clockwise 0-360°)
  float yaw_offset = 0.0;
  float pitch_offset = 0.0;
  float roll_offset = 0.0;

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

  virtual void set_offsets(float yaw = 0.0, float pitch = 0.0, float roll = 0.0) {
    this->yaw_offset = yaw;
    this->pitch_offset = pitch;
    this->roll_offset = roll;
  }
};
