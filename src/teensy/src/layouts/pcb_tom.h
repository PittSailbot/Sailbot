#pragma once
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
 * #27 | NETC7_2 (5V level shift of Water level sensor?)
 * #23 | NETC9_2 (5v level shift of water level sensor 3?)
 *
 * Staggered set of 2x2 pins above teensy in center
 * ? (Teensy SDA2?) | NETC5_2 (?)
 * #9               | NETC8_2 (5v level shift of water level sensor 2?)
 */

// Tom's PCB Pinout Configuration
// PCB -> Teensy pin mappings

// ===== SERVO PINS =====
#define SAIL_SERVO_PIN 9
#define RUDDER_SERVO_PIN 5
// #define JIB_SERVO_PIN        // Not connected on this PCB

// ===== SENSOR PINS =====
#define WINDVANE_ENCODER_A_PIN 34
#define WINDVANE_ENCODER_B_PIN 35

// ===== COMMUNICATION PINS =====
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define TRANSCEIVER_SERIAL &Serial1  // Serial2 wizard shit, Serial1 works through PCB?

// ===== WATER SENSOR ADDRESSES =====
#define WATER_SENSOR1_ADDR 0x36
#define WATER_SENSOR2_ADDR 0x39

// ===== CONTROL PINS =====
#define PUMP1_CONTROL_PIN 28  // AUXFET4

// ===== PCB INFO =====
#define PCB_NAME "Tom PCB"