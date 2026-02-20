#pragma once
#include "teensy.pb.h"

/**
 * @brief Common interface for all GPS implementations
 *
 * This abstract base class defines the minimal interface that all GPS drivers
 * must implement. It provides a consistent API for initialization and data reading
 * regardless of the underlying hardware.
 */
class GPSInterface {
 public:
  bool initialized = false;

  virtual ~GPSInterface() = default;

  /**
   * @brief Initialize the GPS hardware
   * @return true if initialization succeeded, false otherwise
   */
  virtual bool begin() = 0;

  /**
   * @brief Update GPS - call frequently in main loop to read incoming data
   * This method should be called as often as possible to ensure the GPS
   * buffer is being read and NMEA sentences are being assembled.
   */
  virtual void update() = 0;

  /**
   * @brief Read current GPS data
   * @param gps Pointer to GPS protobuf to populate with data
   * @return true if data was successfully read, false otherwise
   */
  virtual bool read(GPSData* gps) = 0;
};
