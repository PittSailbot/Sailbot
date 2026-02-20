#pragma once

#include <SPI.h>

#include "drivers/windvane/windvane.h"

/**
 * @brief Absolute rotary encoder using an SPI interface based on the CALT P3022 Hall Effect
 * Potentiometer
 */
class P3022_WindVane : public WindVaneInterface {
 private:
  const int miso_pin;
  const int cs_pin;
  const int sck_pin;

 public:
  P3022_WindVane(int miso_pin, int cs_pin, int sck_pin);
  bool begin() override;
  bool read(WindVane* windvane) override;
};