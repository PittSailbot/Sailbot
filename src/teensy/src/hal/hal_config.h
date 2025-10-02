#pragma once
#include "components.h"

/**
 * @brief HAL Configuration - Unified platform configurations
 *
 * Each platform defines a complete system: MCU + PCB + Boat configuration
 *
 * Usage:
 * 1. Define your platform (TOM_PCB_DAN_MARINA, etc.)
 * 2. All hardware and feature flags are set automatically
 * 3. Only components with HAS_* = 1 will be compiled into the firmware
 */

// ===== PLATFORM SELECTION =====
// Uncomment ONE of these to select your complete platform configuration

// #define TOM_PCB_DAN_MARINA    // Main boat + Teensy 4.1 + Tom PCB
// #define LEO_PCB_DAN_MARINA   // Mini boat + Pico + Leo PCB
// #define DRAGONFORCE_65      // Mini boat autonomy testing platform
#define PLATFORM_TEST_BREADBOARD  // Cytron RN (TODO: update)

// ===== PLATFORM CONFIGURATIONS =====
/**
 * Define the capabilities and component types for each platform.
 * The rest of the code will selectively compile to include only
 * the enabled modules and their relevant drivers.
 */

#if defined(TOM_PCB_DAN_MARINA)
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
// Hardware platform
#include "layouts/pcb_tom.h"
#define TEENSY41

// Component types - define only what exists, HAS_* flags derived automatically

// Hardware types (type-safe enums)
#define RECEIVER_TYPE ReceiverType::SBUS
#define IMU_TYPE IMUType::ADAFRUIT_BNO055
#define SERVO_TYPE ServoType::ADAFRUIT_I2C_DRIVER
#define PCB_SPEC PCBSpec::TOM_PCB  // Tom's PCB pin layout

#define SAIL_SERVO_SPEC ServoSpec::BILDA
#define RUDDER_SERVO_SPEC ServoSpec::BILDA
// #define JIB_SERVO_SPEC     ServoSpec::HITECH

#define GPS_TYPE GPSType::ADAFRUIT
#define WINDVANE_TYPE WindVaneType::ROTARY_ENCODER
#define WATER_SENSOR_TYPE WaterSensorType::ADAFRUIT_SOIL_SENSOR

#define PLATFORM_NAME "Dan Marina (Tom PCB)"

#elif defined(LEO_PCB_DAN_MARINA)
// Hardware platform
#include "layouts/pcb_leo.h"
#define PICO2

// Component types - define only what exists, HAS_* flags derived automatically
#define RECEIVER_TYPE ReceiverType::IBUS
#define IMU_TYPE IMUType::ADAFRUIT_BNO055
#define SERVO_TYPE ServoType::GPIO
#define PCB_SPEC PCBSpec::LEO_PCB                   // Leo's PCB pin layout

// Servo specifications - define only servos that exist
#define SAIL_SERVO_SPEC ServoSpec::BILDA            // Large BILDA servo for sail
#define RUDDER_SERVO_SPEC ServoSpec::HITECH         // HiTech servo for rudder
#define JIB_SERVO_SPEC ServoSpec::BILDA             // Large BILDA servo for jib

// Optional components - define type if present, comment out if not
#define GPS_TYPE GPSType::ADAFRUIT_PA1616S          // Has GPS
#define WINDVANE_TYPE WindVaneType::ROTARY_ENCODER  // Has windvane
// #define WATER_SENSOR_TYPE                      // No water sensors on this platform

#define PLATFORM_NAME "Dan Marina (Leo PCB)"

#elif defined(DRAGONFORCE_65)
// Hardware platform
#include "layouts/breadboard.h"
#define PICO2

// Component types - define only what exists, HAS_* flags derived automatically
#define MICROCONTROLLER_TYPE MicrocontrollerType::PICO2
#define RECEIVER_TYPE ReceiverType::IBUS
#define IMU_TYPE IMUType::ADAFRUIT_BNO055
#define SERVO_TYPE ServoType::GPIO
#define PCB_SPEC PCBSpec::BREADBOARD  // Breadboard test setup

// Servo specifications - define only servos that exist (minimal for mini boat)
#define SAIL_SERVO_SPEC ServoSpec::JOY880545
#define RUDDER_SERVO_SPEC ServoSpec::JOY881504
// #define JIB_SERVO_SPEC                         // No jib servo on mini boat

// Optional components - minimal setup, most components disabled
// #define GPS_TYPE           // No GPS
// #define WINDVANE_TYPE      // No windvane
// #define WATER_SENSOR_TYPE  // No water sensors

#define PLATFORM_NAME "Mini Boat"

#elif defined(PLATFORM_TEST_BREADBOARD)
// Hardware platform
#include "layouts/breadboard.h"
#define TEENSY41

// Component types - define only what exists, HAS_* flags derived automatically
#define RECEIVER_TYPE ReceiverType::IBUS
// #define IMU_TYPE           IMUType::ADAFRUIT_BNO055
#define SERVO_TYPE ServoType::GPIO
#define PCB_SPEC PCBSpec::BREADBOARD        // Breadboard test setup

// Servo specifications - define only servos that exist (test breadboard has all servos)
#define SAIL_SERVO_SPEC ServoSpec::BILDA    // Test with BILDA servo for sail
#define RUDDER_SERVO_SPEC ServoSpec::BILDA  // Test with BILDA servo for rudder
#define JIB_SERVO_SPEC ServoSpec::BILDA     // Test with BILDA servo for jib

// Optional components - testing platform can have various sensors
// #define GPS_TYPE           GPSType::ADAFRUIT_PA1616S    // Uncomment to test GPS
// #define WINDVANE_TYPE      WindVaneType::ROTARY_ENCODER // Uncomment to test windvane
// #define WATER_SENSOR_TYPE  WaterSensorType::ADAFRUIT_SOIL_SENSOR  // Uncomment to test water
// sensors

#define PLATFORM_NAME "Test Breadboard (Teensy)"

#else
#error \
    "No platform selected. Define a platform in hal_config.h (ex. TOM_PCB_DAN_MARINA, LEO_PCB_DAN_MARINA, DRAGONFORCE_65, PLATFORM_TEST_BREADBOARD, etc.)"
#endif

// ===== DERIVED SETTINGS =====
// Additional definitions that are the same for all platforms

#if defined(TEENSY41)
#define MCU_LED 20  // TODO: add support for MCU light debugging
#elif defined(PICO2) || defined(CYTRON)
#define MCU_LED 25
#endif

#ifdef __CAMERA_SERVOS__
#define CAMERA_SERVOS_I2C 0x40
#endif

// Automatic derived platform capabilities
#ifdef SAIL_SERVO_SPEC
#define HAS_SAIL
#endif

#ifdef RUDDER_SERVO_SPEC
#define HAS_RUDDER
#endif

#ifdef JIB_SERVO_SPEC
#define HAS_JIB
#endif

#ifdef RECEIVER_TYPE
#define HAS_RECEIVER
#endif

#ifdef IMU_TYPE
#define HAS_IMU
#endif

#ifdef GPS_TYPE
#define HAS_GPS
#endif

#ifdef WINDVANE_TYPE
#define HAS_WINDVANE
#endif

#ifdef WATER_SENSOR_TYPE
#define HAS_WATER_SENSORS
#endif