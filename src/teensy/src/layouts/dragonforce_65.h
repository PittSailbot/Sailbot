// Config defining which components and pinouts to use for the HAL platform
#pragma once
#include "hal/components.h"

// ============================================================================
// COMPONENTS
// ============================================================================

#define HAL_MICROCONTROLLER MCU_PICO2
#define HAL_RECEIVER RECEIVER_IBUS
#define HAL_SERVO_TYPE SERVO_PROTOCOL_GPIO
#define HAL_SAIL_SERVO_SPEC ServoSpecs::JOY880545
#define HAL_RUDDER_SERVO_SPEC ServoSpecs::JOY881504
// #define HAL_JIB_SERVO_SPEC       // No jib servo on mini boat
// #define HAL_IMU                  IMU_BNO055
// #define HAL_GPS                  GPS_ADAFRUIT_PA1616S
// #define HAL_WINDVANE             WINDVANE_ROTARY_ENCODER
// #define HAL_WATER_SENSOR         // None

// ============================================================================
// PINOUT CONFIGURATION
// ============================================================================

// Servo pins
#define SAIL_SERVO_PIN 9
#define RUDDER_SERVO_PIN 10
#define JIB_SERVO_PIN 11

// Sensor pins
#define WINDVANE_ENCODER_A_PIN 6
#define WINDVANE_ENCODER_B_PIN 7

// Communication pins
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define TRANSCEIVER_SERIAL &Serial1
#define GPS_SERIAL &Serial2

// Water sensor I2C addresses
#define WATER_SENSOR1_ADDR 0x36
#define WATER_SENSOR2_ADDR 0x39

// ============================================================================
// PLATFORM INFO
// ============================================================================

#define PLATFORM_NAME "Dragonforce 65"