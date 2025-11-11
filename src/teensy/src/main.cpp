// Reads and controls most of the sensors on the boat and interfaces with the Pi via protobuf
#include <Arduino.h>
#include <Wire.h>

#include "elapsedMillis.h"
#include "hal/hal_config.h"
#include "hal/system_factory.h"
#include "pi.pb.h"
#include "protobuf.h"
#include "teensy.pb.h"

// Global platform instance
// Includes all sensors, servos, etc.
std::unique_ptr<Sailbot::SystemFactory> platform;

// Timers
elapsedMillis timer_10HZ;
elapsedMillis timer_1HZ;

// Protobuf data
TeensyData teensy_data = TeensyData_init_default;
PiData pi_data = PiData_init_default;

void setup() {
  // Create the complete platform (MCU + PCB + boat + components)
  platform = std::make_unique<Sailbot::SystemFactory>();
  platform->initialize();

  Serial.printf("I: %s\n", platform->toString().c_str());

  // platform->scanI2C(); // DEBUG I2C WIRING ONLY, this WILL mess with reading sensors
}
#ifdef HAS_SERVOS
void mapControls(RCData* controller) {
  /*Executes the keybind/meaning of each controller input.
    Editing this function will 'rebind' what an input does.

    left_analog_y       - Sail / Camera Pitch
    right_analog_x      - Rudder / Camera Yaw
    right_analog_y      -
    left_analog_x       - Jib
    front_left_switch1  - Up (0): Manual Sail/Rudder      | Mid (1): Manual Rudder  | Down (2):
    Autonomous front_left_switch2  - Up (0):                         | Mid (1): Camera Control |
    Down (2): RC Sail front_right_switch  - Up (0):                         | Mid (1): | Down (2):
    top_left_switch     - Down (0):                       | Up (1):
    top_right_switch    - DISABLED (Reset switch broken)
    potentiometer       -
  */
  // RC / Autonomous Mode
  switch (controller->front_left_switch1) {
    case TRI_SWITCH_UP:  // Manual Sail, Jib & Rudder
      platform->setSail(controller->left_analog_y);
      platform->setRudder(controller->right_analog_x);

#if HAS_JIB_SERVO
      platform->setJib(controller->left_analog_x);
#endif
      break;

    case TRI_SWITCH_MID:  // Manual Rudder, Autonomous Sail & Jib
      if (pi_data.has_cmd_sail) {
        platform->setSail(pi_data.cmd_sail);
      }
#if HAS_JIB_SERVO
      if (pi_data.has_cmd_jib) {
        platform->setJib(pi_data.cmd_jib);
      }
#endif
      platform->setRudder(controller->right_analog_x);
      break;

    case TRI_SWITCH_DOWN:  // Autonomous Sail, Rudder & Jib
      if (pi_data.has_cmd_sail) {
        platform->setSail(pi_data.cmd_sail);
      }
#if HAS_JIB_SERVO
      if (pi_data.has_cmd_jib) {
        platform->setJib(pi_data.cmd_jib);
      }
#endif
      if (pi_data.has_cmd_rudder) {
        platform->setRudder(pi_data.cmd_rudder);
      }
      break;
  }
}
#endif

void loop() {
  if (Serial.available()) {
    readProtobufFromPi(&pi_data);
  }

  teensy_data = TeensyData_init_default;

  if (timer_10HZ > 100) {
    teensy_data.has_rc_data = platform->readControllerState(&teensy_data.rc_data);

#ifdef HAS_SERVOS
    if (teensy_data.has_rc_data) {
      mapControls(&teensy_data.rc_data);
    }

    teensy_data.has_servos = platform->readServos(&teensy_data.servos);
#endif

#ifdef HAS_IMU
    teensy_data.has_imu = platform->imu->read(&teensy_data.imu);
#endif

#ifdef HAS_WINDVANE
    teensy_data.has_windvane = platform->readWindVane(&teensy_data.windvane);
#endif

    timer_10HZ = 0;
  }

  if (timer_1HZ > 1000) {
#ifdef HAS_GPS
    teensy_data.has_gps = platform->gps->read(&teensy_data.gps);
#endif
#if HAS_WATER_SENSORS
    teensy_data.has_water_sensors = platform->checkWaterLevels(&teensy_data.water_sensors);
#endif
    timer_1HZ = 0;
  }

  // Send data to Pi if we have any data to send
  if (teensy_data.has_rc_data || teensy_data.has_gps || teensy_data.has_servos ||
      teensy_data.has_water_sensors || teensy_data.has_windvane || teensy_data.has_camera_servos ||
      teensy_data.has_command) {
    writeProtobufToPi(&teensy_data);
  }

  // Debug output (can be conditionally compiled out later)
  printTeensyProtobuf(&teensy_data);
}