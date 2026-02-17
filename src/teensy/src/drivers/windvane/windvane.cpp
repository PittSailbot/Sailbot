#include "drivers/windvane/windvane.h"

const float RotaryWindVane::FILTER_ALPHA = 0.15f;
RotaryWindVane* RotaryWindVane::instance = nullptr;

RotaryWindVane::RotaryWindVane(uint8_t encoderPinA, uint8_t encoderPinB)
    : pinA(encoderPinA), pinB(encoderPinB), encoderValue(0), 
      hasChanged(false), filteredEncoderValue(0) {
  instance = this;
}

bool RotaryWindVane::begin() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  
  // Initialize filter with current position
  filteredEncoderValue = encoderValue;
  
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, RISING);
  
  Serial.println("I: Started WindVane");
  return true;
}

bool RotaryWindVane::read(WindVane* data) {
  if (hasChanged) {
    int filteredValue = filterValue(encoderValue);
    data->wind_angle = map(filteredValue, 0, ENCODER_ROTATION, 0, 360);
    hasChanged = false;
    return true;
  }
  return false;
}

int RotaryWindVane::filterValue(int rawValue) {
  int diff = rawValue - (int)filteredEncoderValue;
  
  // Handle wrap-around
  if (diff > ENCODER_ROTATION / 2) {
    diff -= ENCODER_ROTATION;
  } else if (diff < -ENCODER_ROTATION / 2) {
    diff += ENCODER_ROTATION;
  }
  
  filteredEncoderValue += FILTER_ALPHA * diff;
  
  // Normalize back to 0-255 range
  if (filteredEncoderValue >= ENCODER_ROTATION) {
    filteredEncoderValue -= ENCODER_ROTATION;
  } else if (filteredEncoderValue < 0) {
    filteredEncoderValue += ENCODER_ROTATION;
  }
  
  return (int)filteredEncoderValue;
}

void RotaryWindVane::encoderISR() {
  if (instance) {
    cli();
    int bState = digitalRead(instance->pinB);
    instance->encoderValue += (bState == HIGH) ? 1 : -1;
    
    // Handle wrap-around
    if (instance->encoderValue >= ENCODER_ROTATION) {
      instance->encoderValue -= ENCODER_ROTATION;
    } else if (instance->encoderValue < 0) {
      instance->encoderValue += ENCODER_ROTATION;
    }
    
    instance->hasChanged = true;
    sei();
  }
}