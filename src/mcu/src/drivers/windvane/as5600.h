#pragma once

#include <Adafruit_AS5600.h>
#include <Wire.h>

#include "drivers/windvane/windvane.h"

/**
 * @brief Driver for Adafruit AS5600 Magnetic Rotary Encoder (WindVane)
 *
 * Provides a 12-bit absolute angle measurement via I2C.
 *
 * @see https://github.com/adafruit/Adafruit_AS5600
 */
class AS5600_WindVane : public WindVaneInterface {
 private:
  Adafruit_AS5600 as5600;

 public:
  AS5600_WindVane();

  bool begin() override;
  bool read(WindVane* windvane) override;
};
