// Test/Breadboard PCB Configuration
// Simple pin assignments for breadboard testing and development
#pragma once

// ===== SERVO PINS =====
#define SAIL_SERVO_PIN 9
#define RUDDER_SERVO_PIN 10
#define JIB_SERVO_PIN 11

// ===== SENSOR PINS =====
#define WINDVANE_ENCODER_A_PIN 6
#define WINDVANE_ENCODER_B_PIN 7

// ===== COMMUNICATION PINS =====
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define TRANSCEIVER_SERIAL &Serial1

// ===== WATER SENSOR ADDRESSES =====
#define WATER_SENSOR1_ADDR 0x36
#define WATER_SENSOR2_ADDR 0x39

// ===== PCB INFO =====
#define PCB_NAME "Breadboard"