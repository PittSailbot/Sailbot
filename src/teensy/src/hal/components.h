#pragma once
#include <stdint.h>

#include <optional>

/**
 * @brief HAL Configuration - Hardware declarations
 *
 * If we get a new sensor, servo, receiver, PCB, etc. add it below!
 */

enum class MicrocontrollerType { PICO2, CYTRON, TEENSY41 };

// Which RC receiver (and it's corresponding controller scheme) is used
enum class ReceiverType {
  SBUS,  // Any generic SBUS receiver (currently our FrSky X9-lite)
  IBUS,  // Any generic IBUS receiver (currently our FS-IA6B)
};

// NOT IMPLEMENTED: In case we ever want to mix and match controllers
// enum class ControllerType {
//     FRSKY_X9,
//     FLYSKY_FSI6X
// }

enum class IMUType {
  ADAFRUIT_BNO055,          // Adafruit BNO055 9-DOF
  ADAFRUIT_LSM6DS_LIS3MDL,  // LSM6DS + LIS3MDL combo
};

// How each servo's PWM signal is sent
enum class ServoType {
  GPIO,                 // Direct GPIO PWM servos
  ADAFRUIT_I2C_DRIVER,  // I2C servo driver board (PCA9685)
};

// Defines the relevant microcontroller pinouts to each component
enum class PCBSpec {
  TOM_PCB,     // Tom's PCB layout
  LEO_PCB,     // Leo's PCB layout
  BREADBOARD,  // Breadboard test setup
  CUSTOM       // Custom pin assignments
};

// GPS module types
enum class GPSType {
  ADAFRUIT_PA1616S,  // Adafruit Ultimate GPS https://www.adafruit.com/product/746
  ADAFRUIT_PA1010D   // Mini GPS w/ I2C & UART https://www.adafruit.com/product/4415
};

// WindVane types
enum class WindVaneType {
  ROTARY_ENCODER  // Rotary encoder windvane
};

// Water sensor types
enum class WaterSensorType {
  ADAFRUIT_SOIL_SENSOR,    // Capactive sensor communicating over I2C
                           // https://www.adafruit.com/product/4026
  ADAFRUIT_DIGITAL_SENSOR  // (these suck, don't bother using) https://www.adafruit.com/product/4965
};

// Defines the min/max PWM and min/max angles for each servo type
enum class ServoSpec {
  BILDA,      // Bilda servos (sails)
              // https://www.servocity.com/2000-series-5-turn-dual-mode-servo-25-2-torque/
  HITECH,     // HiTech servos  (rudder) https://www.servocity.com/d845wp-servo/?sku=3684500
  JOY880545,  // Mini boat servo winch
              // https://radiosailing.net/collections/dragonforce-65-v6-replacement-parts/products/sail-winch-servo-set-df65
  JOY881504,  // Mini boat servo rudder
              // https://radiosailing.net/collections/dragonforce-65-v6-replacement-parts/products/new-digital-metal-gear-rudder-servo_2025
};

// Servo specification data structure
struct ServoSpecData {
  uint16_t min_pwm;
  uint16_t max_pwm;
  uint16_t min_angle;
  uint16_t max_angle;
  const char* name;
};

// Servo specification lookup table
constexpr ServoSpecData getServoSpec(ServoSpec spec) {
  switch (spec) {
    case ServoSpec::BILDA:
      return {500, 2500, 0, 1800, "BILDA"};
    case ServoSpec::HITECH:
      return {870, 2320, 0, 146, "HiTech"};
    case ServoSpec::JOY880545:
      return {1520, 2500, 0, 612,
              "JoySway Winch"};  // TODO: Guestimating the PWM/angle range, no specs online
    case ServoSpec::JOY881504:
      return {1520, 2500, 0, 180,
              "JoySway Servo"};  // TODO: Guestimating the PWM/angle range, no specs online
  }
  return {1000, 2000, 0, 180, "Default"};  // Fallback value for undefined servos
};