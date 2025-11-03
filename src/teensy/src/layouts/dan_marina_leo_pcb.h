// Config defining which components and pinouts to use for the HAL platform
#pragma once
#include "hal/components.h"

// ============================================================================
// COMPONENTS
// ============================================================================

#define HAL_MICROCONTROLLER MCU_PICO2
#define HAL_RECEIVER RECEIVER_SBUS
#define HAL_SERVO_TYPE SERVO_PROTOCOL_GPIO
#define HAL_SAIL_SERVO_SPEC ServoSpecs::BILDA
#define HAL_RUDDER_SERVO_SPEC ServoSpecs::HITECH
#define HAL_JIB_SERVO_SPEC ServoSpecs::BILDA
#define HAL_IMU IMU_BNO055
#define HAL_GPS GPS_ADAFRUIT_PA1616S
#define HAL_WINDVANE WINDVANE_ROTARY_ENCODER
// #define HAL_WATER_SENSOR         // None

// ============================================================================
// PINOUT CONFIGURATION
// ============================================================================

// Servo pins
#define SAIL_SERVO_PIN 23
#define RUDDER_SERVO_PIN 9
#define JIB_SERVO_PIN 24

// Sensor pins
#define WINDVANE_ENCODER_A_PIN 10
#define WINDVANE_ENCODER_B_PIN 11

// Communication pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define TRANSCEIVER_SERIAL &Serial1
#define GPS_SERIAL &Serial2

// Water sensor I2C addresses
#define WATER_SENSOR1_ADDR 0x36
#define WATER_SENSOR2_ADDR 0x39

// ============================================================================
// PLATFORM INFO
// ============================================================================

#define PLATFORM_NAME "Dan Marina (Leo PCB)"