// Config defining which components and pinouts to use for the HAL platform
#pragma once
#include "hal/components.h"

// ============================================================================
// COMPONENTS
// ============================================================================

#define HAL_MICROCONTROLLER MCU_PICO2
#define HAL_RECEIVER RECEIVER_SBUS
#define HAL_SERVO_TYPE SERVO_PROTOCOL_GPIO
#define HAL_SAIL_SERVO_SPEC ServoSpecs::JOY880545
#define HAL_RUDDER_SERVO_SPEC ServoSpecs::DEFAULT
// #define HAL_JIB_SERVO_SPEC       // No jib servo on mini boat
#define HAL_IMU IMU_BNO055
#define HAL_GPS GPS_ADAFRUIT_PA1010D
#define HAL_WINDVANE WINDVANE_P3022
// #define HAL_WATER_SENSOR         // None

// ============================================================================
// PINOUT CONFIGURATION
// ============================================================================

// Servo pins
#define SAIL_SERVO_PIN 14    // S3
#define RUDDER_SERVO_PIN 15  // S2
#define JIB_SERVO_PIN 16     // S1

// Sensor pins
#define WINDVANE_MISO_PIN 21
#define WINDVANE_CS_PIN 17
#define WINDVANE_SCK_PIN 18

// Communication pins
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21
#define TRANSCEIVER_RX_PIN 1

// Debug LED
#define LED_PIN 25  // Onboard Pico2 LED (does not work on Pico2W)

// ============================================================================
// SENSOR CONFIG
// ============================================================================

// For the IMU to work it MUST be aligned with the boat's physical orientation
// Use these offsets to account for static differences from mounting/PCB layout
// For best accuracy, mount the PCB in the orientation in the boat to avoid recalibrating
// Angles are in 0-360° clockwise
// For the BNO055 (front 0° aligns with the y-silkscreen arrow)
#define IMU_YAW_OFFSET \
  270  // Should be close to a multiple of of 90° (depends on which way IMU is flipped)
#define IMU_ROLL_OFFSET \
  0  // Should be close to 0 (depends if PCB is level with boat deck or IMU is upside down)
#define IMU_PITCH_OFFSET 0  // Should be close to 0 (depends if PCB is tilted front/back from deck)

// ============================================================================
// PLATFORM INFO
// ============================================================================

#define PLATFORM_NAME "Pico2 Protoboard"