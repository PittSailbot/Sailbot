#pragma once
#include <stdint.h>

#include <optional>

/**
 * @brief HAL Configuration - Hardware declarations
 *
 * If we get a new sensor, servo, receiver, PCB, etc. add it below!
 */

// Unfortunately enums/constexpr don't work with the preprocessor so no real type-safe way to
// conditionally include driver-specific headers

// ===== MICROCONTROLLERS =====
#define MCU_PICO2 1
#define MCU_CYTRON 2
#define MCU_TEENSY41 3

// ===== RC RECEIVERS =====
#define RECEIVER_SBUS 1  // Any generic SBUS receiver (currently our FrSky X9-lite)
#define RECEIVER_IBUS 2  // Any generic IBUS receiver (currently our FS-IA6B)

// ===== CONTROLLERS =====
// NOT IMPLEMENTED: In case we ever want to mix and match controllers
// #define CONTROLLER_FRSKY_X9
// #define FLYSKY_FSI6X

// ===== IMUs =====
// Preprocessor-compatible constants for conditional compilation
#define IMU_BNO055 1  // Adafruit BNO055 9-DOF
#define IMU_LSM6DS 2  // LSM6DS + LIS3MDL combo

// ===== GPSs =====
#define GPS_ADAFRUIT_PA1616S 1  // Adafruit Ultimate GPS https://www.adafruit.com/product/746
#define GPS_ADAFRUIT_PA1010D 2  // Mini GPS w/ I2C & UART https://www.adafruit.com/product/4415

// ===== WINDVANES =====
#define WINDVANE_ROTARY_ENCODER 1

// ===== WATER SENSORS =====
#define WATER_SENSOR_ADAFRUIT_SOIL \
  1  // Capactive sensor communicating over I2C  // Capactive sensor communicating over I2C
#define WATER_SENSOR_ADAFRUIT_DIGITAL \
  2  // (these suck, don't bother using) https://www.adafruit.com/product/4965

// ===== SERVO COMMUNICATION PROTOCOL =====
// How each servo's PWM signal is sent; currently no way to mix/match GPIO & I2C servos
#define SERVO_PROTOCOL_GPIO 1          // Direct GPIO PWM servos
#define SERVO_PROTOCOL_ADAFRUIT_I2C 2  // I2C servo driver board (PCA9685)

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
constexpr ServoSpecData JOY880545 = {1000, 2000, 612, "JoySway Winch"};
// Mini boat servo (rudder)
// https://radiosailing.net/collections/dragonforce-65-v6-replacement-parts/products/new-digital-metal-gear-rudder-servo_2025
constexpr ServoSpecData JOY881504 = {1000, 2000, 60, "JoySway Servo"};
constexpr ServoSpecData DEFAULT = {1000, 2000, 180, "Default"};
}  // namespace ServoSpecs