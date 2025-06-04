#include "protobuf.h"

#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <sstream>

#include "transceiver.h"

uint8_t teensy_buffer[TEENSY_PB_H_MAX_SIZE];
uint8_t pi_buffer[PI_PB_H_MAX_SIZE];

void readProtobufFromPi(PiData* pi_data) {
  pb_istream_t istream = pb_istream_from_buffer(pi_buffer, sizeof(pi_buffer));
  memset(teensy_buffer, 0, sizeof(teensy_buffer));
  bool status = pb_decode(&istream, PiData_fields, pi_data);

  if (!status) {
    Serial.printf("Failed to read protobuf from Pi: %s\n", istream.errmsg);
  }
  return;
}

void writeProtobufToPi(TeensyData* teensy_data) {
  pb_ostream_t ostream = pb_ostream_from_buffer(teensy_buffer, sizeof(teensy_buffer));
  memset(teensy_buffer, 0, sizeof(teensy_buffer));
  bool status = pb_encode(&ostream, TeensyData_fields, teensy_data);

  if (!status) {
    Serial.printf("E: Failed to write protobuf to Pi: %s\n", ostream.errmsg);
    return;
  }

  if (ostream.bytes_written > 0) {
    Serial.write(teensy_buffer, ostream.bytes_written);
    Serial.write("\n");
    // Serial.printf(
    //     "V: Wrote protobuf to Pi:\
    // RC: %d\
    // GPS: %d\
    // IMU: %d\
    // Servos: %d\
    // Water Sensors: %d\
    // Windvane: %d\
    // Camera Servos: %d\
    // Command: %d\n\
    //   ",
    //     teensy_data->has_rc_data, teensy_data->has_gps, teensy_data->has_imu,
    //     teensy_data->has_servos, teensy_data->has_water_sensors, teensy_data->has_windvane,
    //     teensy_data->has_camera_servos, teensy_data->has_command);
  }
}
