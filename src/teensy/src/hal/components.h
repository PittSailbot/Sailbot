#pragma once
#include <stdint.h>

#include <optional>

/**
 * @brief HAL Configuration - Hardware declarations
 *
 * If we get a new sensor, servo, receiver, PCB, etc. add it below!
 */

// ===== MICROCONTROLLERS =====
enum class MicrocontrollerType { PICO2, CYTRON, TEENSY41 };

// ===== PCB PINOUT =====
// Defines the relevant microcontroller pinouts to each component
enum class PCBSpec {
  TOM_PCB,     // The double-sided blue one
  LEO_PCB,     // The single-sided green one
  BREADBOARD,  // Breadboard test setup
};

// ===== RC RECEIVERS =====
enum class ReceiverType {
  SBUS,  // Any generic SBUS receiver (currently our FrSky X9-lite)
  IBUS,  // Any generic IBUS receiver (currently our FS-IA6B)
};

// ===== CONTROLLERS =====
// NOT IMPLEMENTED: In case we ever want to mix and match controllers
// enum class ControllerType {
//     FRSKY_X9,
//     FLYSKY_FSI6X
// }

// ===== IMUs =====
enum class IMUType {
  ADAFRUIT_BNO055,          // Adafruit BNO055 9-DOF
  ADAFRUIT_LSM6DS_LIS3MDL,  // LSM6DS + LIS3MDL combo
};

// ===== GPSs =====
enum class GPSType {
  ADAFRUIT_PA1616S,  // Adafruit Ultimate GPS https://www.adafruit.com/product/746
  ADAFRUIT_PA1010D   // Mini GPS w/ I2C & UART https://www.adafruit.com/product/4415
};

// ===== WINDVANES =====
enum class WindVaneType { ROTARY_ENCODER };

// ===== WATER SENSORS =====
enum class WaterSensorType {
  ADAFRUIT_SOIL_SENSOR,    // Capactive sensor communicating over I2C
                           // https://www.adafruit.com/product/4026
  ADAFRUIT_DIGITAL_SENSOR  // (these suck, don't bother using) https://www.adafruit.com/product/4965
};

// ===== SERVO COMMUNICATION PROTOCOL =====
// How each servo's PWM signal is sent; currently no way to mix/match GPIO & I2C servos
enum class ServoType {
  GPIO,                 // Direct GPIO PWM servos
  ADAFRUIT_I2C_DRIVER,  // I2C servo driver board (PCA9685)
};

// ===== SERVOS =====
// Define new servos here!
struct ServoSpecData {
  uint16_t min_pwm;
  uint16_t max_pwm;
  uint16_t max_angle;
  const char* name;
};

namespace ServoSpecs {
// Bilda servos (sails) https://www.servocity.com/2000-series-5-turn-dual-mode-servo-25-2-torque/
constexpr ServoSpecData BILDA = {500, 2500, 1800, "BILDA"};
// HiTech servos  (rudder) https://www.servocity.com/d845wp-servo/?sku=3684500
constexpr ServoSpecData HITECH = {870, 2320, 146, "HiTech"};
// Mini boat servo winch (sails)
// https://radiosailing.net/collections/dragonforce-65-v6-replacement-parts/products/sail-winch-servo-set-df65
constexpr ServoSpecData JOY880545 = {
    1520, 2500, 612, "JoySway Winch"};  // TODO: Guestimating the PWM/angle range, no specs online
// Mini boat servo (rudder)
// https://radiosailing.net/collections/dragonforce-65-v6-replacement-parts/products/new-digital-metal-gear-rudder-servo_2025
constexpr ServoSpecData JOY881504 = {
    1520, 2500, 180, "JoySway Servo"};  // TODO: Guestimating the PWM/angle range, no specs online
constexpr ServoSpecData DEFAULT = {1000, 2000, 180, "Default"};
}  // namespace ServoSpecs