// Config defining which components and pinouts to use for the HAL platform
/*
Currently retrofitted PCB from 'She Got the Kids' that we've been trying to make work on Dan Marina
It was built to use Odrive DC motor encoders with that code being on the Pi5, except
we've retrofitted it to work with servos. The Teensy 4.1 breakouts are kind of spread out
which led to issues with the PCB being soldered to the servos in the boat, Dupont connector
conversions shorting internally, and lots of other problems. Using the adafruit servo driver
to send PWM to the servos over I2C and powering the servos directly over the 5V trace (DON't use
the Adafruit 5V/GND it can't handle it) has kind of worked but the wiring is very intermittent
and sometimes just stops working.
*/
#pragma once
#include "hal/components.h"

// ============================================================================
// COMPONENTS
// ============================================================================

#define HAL_MICROCONTROLLER MCU_TEENSY41
#define HAL_RECEIVER RECEIVER_SBUS
#define HAL_SERVO_TYPE SERVO_PROTOCOL_GPIO
#define HAL_SAIL_SERVO_SPEC \
  ServoSpecs::HITECH  // TODO BUGFIX BILDA ONLY MOVES VERY SHORTLY (Cconstrain?)
#define HAL_RUDDER_SERVO_SPEC ServoSpecs::HITECH
// #define HAL_JIB_SERVO_SPEC          ServoSpecs::BILDA
// #define HAL_IMU                     IMU_BNO055
// #define HAL_GPS                     GPS_ADAFRUIT_PA1616S
// #define HAL_WINDVANE                WINDVANE_ROTARY_ENCODER
// #define HAL_WATER_SENSOR            WATER_SENSOR_ADAFRUIT_SOIL

// ============================================================================
// PINOUT CONFIGURATION
// ============================================================================
/**
 * Pinouts for Tom's PCB (the blue one). He's since graduated and combing through his pcb
 * schematic hasn't been the most clear, so these pinouts may not be 100% correct or
 * all-inclusive of what's actually on the pcb.
 *
 * However, these are the rough geographic locations of each pinout on the Teensy 4.1:
 *
 * AUXFETS (screw terminals) in order from edge to usb-c on pcb:
 * 12V | #38 (Connected to Teensy 5v but not 5v?) | #2 (30? 31?) | #29 | #28 | ? | ? | 12V
 *
 * Pins repurposed from water level gpio, set of 6 pins right above Teensy
 * in order from PCB center to edge:
 * GND? | #39 | #40 | #41 | GND | 3.3V
 *
 * Set of 2x2 pins on the edge of pcb near teensy, in order:
 * #24 | #24
 * ? | #23
 *
 * Staggered set of 2x2 pins above teensy in center
 * ? (Teensy SDA2?) | NETC5_2 (?)
 * #9               | NETC8_2 (5v level shift of water level sensor 2?)
 */

// Servo pins
#define SAIL_SERVO_PIN 23
#define RUDDER_SERVO_PIN 24
// #define JIB_SERVO_PIN               11

// Sensor pins
#define WINDVANE_ENCODER_A_PIN 34
#define WINDVANE_ENCODER_B_PIN 35

// Communication pins
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define TRANSCEIVER_SERIAL &Serial1

// Water sensor I2C addresses
#define WATER_SENSOR1_ADDR 0x36
#define WATER_SENSOR2_ADDR 0x39

// ===== CONTROL PINS =====
#define PUMP1_CONTROL_PIN 28  // AUXFET4

// ============================================================================
// PLATFORM INFO
// ============================================================================

#define PLATFORM_NAME "Dan Marina (Tom PCB)"