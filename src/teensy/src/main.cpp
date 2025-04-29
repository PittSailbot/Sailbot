// Main program running on the Teensy
// Reads and controls most of the sensors on the boat and interfaces with the Pi via protobuf
#include <Arduino.h>
// #include <ArduinoLog.h>
#include <IntervalTimer.h>
#include <Wire.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <sbus.h>

#include "gps.h"
#include "imu.h"
#include "pi.pb.h"
#include "servos.h"
#include "teensy.h"
#include "teensy.pb.h"
#include "transceiver.h"
#include "water_sensors.h"
#include "windvane.h"

// Uncomment to disable all logging. Or use Log.setLevel() to hide low priority logs.
// #define DISABLE_LOGGING

int pwm_val = 0;
int pwm_peak = 150;

IntervalTimer filterTimer;

TeensyData teensy_data = TeensyData_init_default;
PiData pi_data = PiData_init_default;

void readProtobufFromPi(PiData* pi_data) {
  if (Serial.available()) {
    uint8_t buffer[PI_PB_H_MAX_SIZE];
    pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_decode(&stream, PiData_fields, pi_data);

    if (!status) {
      // Log.errorln("Failed to read protobuf from Pi: %s", stream.errmsg);
    }
    return;
  }
}

void writeProtobufToPi(TeensyData* teensy_data) {
  uint8_t buffer[TEENSY_PB_H_MAX_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_encode(&stream, TeensyData_fields, teensy_data);

  if (status) {
    Serial.write(buffer, stream.bytes_written);
    Serial.println();
  } else {
    // Log.errorln("Failed to write protobuf to Pi: %s", stream.errmsg);
  }
}

void setup() {
  // Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  // while (!Serial) {}
  setupTransceiver();
  setupWindVane();
  // setupGPS();
  setupIMU();
  setupServos();
  if (!filterTimer.begin(updateIMU, int(1000000 / FILTER_UPDATE_RATE_HZ))) {
    // Log.errorln("Failed to start filter timer");
  }
  setupWaterSensors();
  setupPumps();
  // setupReceiver();

  // Log.infoln("Initialized Teensy");
}

void loop() {
  teensy_data = TeensyData_init_default;
  teensy_data.has_rc_data = readControllerState(&teensy_data.rc_data);
  teensy_data.has_windvane = readWindVane(&teensy_data.windvane);
  teensy_data.has_gps = readGPS(&teensy_data.gps);
  teensy_data.has_imu = readIMU(&teensy_data.imu);
  teensy_data.has_water_sensors = readWaterSensors(&teensy_data.water_sensors);
  teensy_data.has_servos = readServos(&teensy_data.servos);

  if (Serial.available()) {
    readProtobufFromPi(&pi_data);
  }

  writeProtobufToPi(&teensy_data);

  delay(5000);
}
