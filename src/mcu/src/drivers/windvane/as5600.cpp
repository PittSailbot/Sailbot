// Driver for Adafruit AS5600 12-bit Contactless Magnetic Rotary Encoder (WindVane)
#include "drivers/windvane/as5600.h"

#include <Adafruit_AS5600.h>
#include <Wire.h>

AS5600_WindVane::AS5600_WindVane() {
}

bool AS5600_WindVane::begin() {
  if (!this->as5600.begin()) {
    Serial.println("E: Failed to find AS5600 WindVane... Check your wiring or I2C ADDR!");
    return false;
  }

  // The AS5600 needs a magnetic field to function correctly
  // (Optional: but good to verify the magnet is placed correctly)
  // if (!this->as5600.isMagnetDetected()) {
  //   Serial.println("W: AS5600 WindVane initialized, but no magnet detected. Check magnet
  //   placement!");
  // }

  this->initialized = true;
  return true;
}

bool AS5600_WindVane::read(WindVane* windvane) {
  if (!this->initialized) {
    Serial.println("W: Trying to read from uninitialized AS5600 WindVane");
    return false;
  }
  uint8_t agc = this->as5600.getAGC();
  if (agc >= 128) {
    // The sensor can actually read magnets further than this allows
    // However, any further away and we can't differentiate real from bad data
    // If mounting is secure & it still reads reliably, you can disable this to raise sensitivity
    Serial.println("W: AS5600 Magnet is too weak or too far");
    return false;
  }

  // The AS5600 measures angles from 0 to 4095 (12-bit)
  int32_t degrees = (as5600.getRawAngle() * 360) / 4096;

  windvane->wind_angle = degrees;

  return true;
}
