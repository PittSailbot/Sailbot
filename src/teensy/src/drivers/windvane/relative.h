#pragma once

#include <Arduino.h>

#include "drivers/windvane/windvane.h"
#include "teensy.pb.h"

/**
 * @brief Any relative rotary encoder using an A/B GPIO pins
 * Note: if this type of sensor is used, it will have to be oriented with the front of the boat
 * every time
 */
class Relative_WindVane : public WindVaneInterface {
 private:
  uint8_t pinA;
  uint8_t pinB;

  volatile int encoderValue;
  volatile bool hasChanged;
  float filteredEncoderValue;

  static const int ENCODER_ROTATION = 256;
  static const float FILTER_ALPHA;

  // Static instance pointer for ISR
  static Relative_WindVane* instance;

  int filterValue(int rawValue);
  static void encoderISR();

 public:
  Relative_WindVane(uint8_t encoderPinA, uint8_t encoderPinB);
  bool begin();
  bool read(WindVane* data);
};