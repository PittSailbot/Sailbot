#pragma once
/**
 * Pinouts for Leo's PCB (the single-sided green one).
 *
 * However, these are the rough geographic locations of each pinout on the Teensy 4.1:
 *
 * TODO: Add geographic reference of MCU pinouts on PCB
 */

// Leo's PCB Pinout Configuration
// PCB -> Pico pin mappings

// ===== SERVO PINS =====
#define SAIL_SERVO_PIN 23
#define RUDDER_SERVO_PIN 9
#define JIB_SERVO_PIN 24

// ===== SENSOR PINS =====
#define WINDVANE_ENCODER_A_PIN 10
#define WINDVANE_ENCODER_B_PIN 11

// ===== COMMUNICATION PINS =====
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define TRANSCEIVER_SERIAL &Serial1

// ===== WATER SENSOR ADDRESSES =====
#define WATER_SENSOR1_ADDR 0x36
#define WATER_SENSOR2_ADDR 0x39

// ===== PCB INFO =====
#define PCB_NAME "Leo PCB"