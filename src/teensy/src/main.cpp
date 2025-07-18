// Main program running on the Teensy
// Reads and controls most of the sensors on the boat and interfaces with the Pi via protobuf
#include <Arduino.h>
#include <I2CScanner.h>
#include <IntervalTimer.h>
#include <Wire.h>
#include <sbus.h>

#include "camera_servos.h"
#include "elapsedMillis.h"
#include "gps.h"
#include "imu.h"
#include "pi.pb.h"
#include "protobuf.h"
#include "servos.h"
#include "teensy.h"
#include "teensy.pb.h"
#include "transceiver.h"
#include "water_sensors.h"
#include "windvane.h"

IntervalTimer filterTimer;
elapsedMillis timer_20HZ;
elapsedMillis timer_10HZ;
elapsedMillis timer_1HZ;

TeensyData teensy_data = TeensyData_init_default;
PiData pi_data = PiData_init_default;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  // while (!Serial) {}
  setupTransceiver();
  setupWindVane();
  if (!GPS_USE_PI) {
    setupGPS();
  }
  setupIMU();
  setupServos();
  setupWaterSensors();
  setupPumps();
  // setupReceiver();

  Serial.println("I: Initialized Teensy");

  I2CScanner scanner;
  scanner.Init();
  scanner.Scan();
}

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
      setSail(controller->left_analog_y);
      setJib(controller->left_analog_x);
      setRudder(controller->right_analog_x);
      break;
    case TRI_SWITCH_MID:  // Manual Rudder, Autonomous Sail & Jib
      if (pi_data.has_cmd_sail) {
        setSail(pi_data.cmd_sail);
      }
      if (pi_data.has_cmd_jib) {
        setJib(pi_data.cmd_jib);
      }
      setRudder(controller->right_analog_x);
      break;
    case TRI_SWITCH_DOWN:  // Autonomous Sail, Rudder & Jib
      if (pi_data.has_cmd_sail) {
        setSail(pi_data.cmd_sail);
      }
      if (pi_data.has_cmd_jib) {
        setJib(pi_data.cmd_jib);
      }
      if (pi_data.has_cmd_rudder) {
        setRudder(pi_data.cmd_rudder);
      }
      break;
  }
}

void loop() {
  if (Serial.available()) {
    readProtobufFromPi(&pi_data);
  }

  teensy_data = TeensyData_init_default;
  if (timer_20HZ > 500) {
    teensy_data.has_rc_data = readControllerState(&teensy_data.rc_data);

    if (&teensy_data.has_rc_data) {
      mapControls(&teensy_data.rc_data);
    }
    teensy_data.has_servos = readServos(&teensy_data.servos);
    timer_20HZ = 0;
  }
  if (timer_10HZ > 1000) {
    if (!GPS_USE_PI) {
      teensy_data.has_gps = readGPS(&teensy_data.gps);
    }
    teensy_data.has_windvane = readWindVane(&teensy_data.windvane);
    teensy_data.has_imu = readIMU(&teensy_data.imu);
    // teensy_data.has_camera_servos = readCameraServos(&teensy_data.camera_servos);
    timer_10HZ = 0;
  }
  if (timer_1HZ > 10000) {
    teensy_data.has_water_sensors = readWaterSensors(&teensy_data.water_sensors);
    timer_1HZ = 0;
  }

  if (&teensy_data.has_rc_data || &teensy_data.has_gps || &teensy_data.has_imu ||
      &teensy_data.has_servos || &teensy_data.has_water_sensors || &teensy_data.has_windvane ||
      &teensy_data.has_camera_servos || &teensy_data.has_command) {
    writeProtobufToPi(&teensy_data);
  }

  printTeensyProtobuf(&teensy_data);
}