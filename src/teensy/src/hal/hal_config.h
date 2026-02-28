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

//  #define DEBUG // Uncomment to enable debug mode

// ===== PLATFORM SELECTION =====
// Uncomment ONE of these to select your complete platform configuration

// #define DAN_MARINA_TOM_PCB  // Main boat + Teensy 4.1 + Tom PCB
// #define DAN_MARINA_LEO_PCB   // Mini boat + Pico + Leo PCB
// #define DRAGONFORCE_65      // Mini boat autonomy testing platform
// #define PROTOBOARD            // Prototype board for Mini Boat PCB
#define BREADBOARD

// ===== PLATFORM CONFIGURATIONS =====
/**
 * Define the capabilities and component types for each platform.
 * The rest of the code will selectively compile to include only
 * the enabled modules and their relevant drivers.
 */
#if defined(DAN_MARINA_TOM_PCB)
#include "layouts/dan_marina_tom_pcb.h"

#elif defined(DAN_MARINA_LEO_PCB)
#include "layouts/dan_marina_leo_pcb.h"

#elif defined(DRAGONFORCE_65)
#include "layouts/dragonforce_65.h"

#elif defined(PROTOBOARD)
#include "layouts/protoboard.h"

#elif defined(BREADBOARD)
#include "layouts/breadboard.h"

#else
#error \
    "No platform selected. Define a platform in hal_config.h (ex. DAN_MARINA_TOM_PCB, DAN_MARINA_LEO_PCB, DRAGONFORCE_65, BREADBOARD, etc.)"
#endif

// ===== DERIVED SETTINGS =====
// Additional definitions that are the same for all platforms

#if defined(PLATFORM_TEENSY)
#define MCU_LED 20
#elif defined(PLATFORM_RP2350) || defined(PLATFORM_CYTRON)
#define MCU_LED 25
#endif

// Automatic derived platform capabilities
#ifdef HAL_SAIL_SERVO_SPEC
#define HAS_SAIL
#endif

#ifdef HAL_RUDDER_SERVO_SPEC
#define HAS_RUDDER
#endif

#ifdef HAL_JIB_SERVO_SPEC
#define HAS_JIB
#endif

#if defined(HAS_SAIL) || defined(HAS_RUDDER) || defined(HAS_JIB)
#define HAS_SERVOS
#endif

#ifdef HAL_RECEIVER
#define HAS_RECEIVER
#endif

#ifdef HAL_IMU
#define HAS_IMU
#endif

#ifdef HAL_GPS
#define HAS_GPS
#endif

#ifdef HAL_WINDVANE
#define HAS_WINDVANE
#endif

#ifdef HAL_WATER_SENSOR
#define HAS_WATER_SENSORS
#endif

#ifdef LED_PIN
#define HAS_LED
#endif