// Config defining which components and pinouts to use for the HAL platform
#pragma once
#include "hal/components.h"

// ============================================================================
// COMPONENTS
// ============================================================================

#define HAL_MICROCONTROLLER MCU_PICO2
#define HAL_RECEIVER RECEIVER_SBUS
#define HAL_SERVO_TYPE SERVO_PROTOCOL_GPIO
#define HAL_SAIL_SERVO_SPEC ServoSpecs::DEFAULT
#define HAL_RUDDER_SERVO_SPEC ServoSpecs::JOY881504
// #define HAL_JIB_SERVO_SPEC ServoSpecs::DEFAULT
#define HAL_IMU IMU_BNO055
#define HAL_GPS GPS_ADAFRUIT_PA1010D
#define HAL_WINDVANE WINDVANE_P3022
// #define HAL_WATER_SENSOR         // None

// ============================================================================
// PINOUT CONFIGURATION
// ============================================================================

// Servo pins
#define SAIL_SERVO_PIN 6    // MS 5-7.5V GP4
#define RUDDER_SERVO_PIN 5  // RS 5-7.5V GP5
// #define JIB_SERVO_PIN 6     // JS 5-7.5V GP6

// A1 GP28 5-7.5V
// A2 GP8 5v
// A3 GP22 5v

// Sensor pins
#define WINDVANE_MOSI_PIN 19  // Should be connected to 3.3V or GP19
#define WINDVANE_MISO_PIN 16 //GP16
#define WINDVANE_CS_PIN 17 //GP17
#define WINDVANE_SCK_PIN 18 //GP18

// JST-SH
// 3.3V
// I2C SDA GP0
// I2C SCL GP1
// GND

// Communication pins
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define TRANSCEIVER_RX_PIN 9

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

#define PLATFORM_NAME "Pico2 Oscar PCB"