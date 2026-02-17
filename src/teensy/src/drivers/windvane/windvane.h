#pragma once

#include "teensy.pb.h"
#include <Arduino.h>

class RotaryWindVane {
 private:
  uint8_t pinA;
  uint8_t pinB;
  
  volatile int encoderValue;
  volatile bool hasChanged;
  float filteredEncoderValue;
  
  static const int ENCODER_ROTATION = 256;
  static const float FILTER_ALPHA;
  
  // Static instance pointer for ISR
  static RotaryWindVane* instance;
  
  int filterValue(int rawValue);
  static void encoderISR();

 public:
  RotaryWindVane(uint8_t encoderPinA, uint8_t encoderPinB);
  bool begin();
  bool read(WindVane* data);
};