#pragma once
#include <Arduino.h>

#include "teensy.pb.h"

/**
 * @brief Base template class for windvane/rotary-encoder implementations
 */
class WindVaneInterface {
 public:
  bool initialized = false;

  virtual ~WindVaneInterface() = default;

  /**
   * @brief Initialize the encoder
   * @return true on success, false on fail
   */
  virtual bool begin() = 0;

  /**
   * @brief Read the current encoder data
   * @param windvane pointer to WindVane protobuf to populate with data
   * @return true if valid data was read, false otherwise
   */
  virtual bool read(WindVane* windvane) = 0;
};