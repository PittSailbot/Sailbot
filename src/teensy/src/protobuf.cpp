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

void printTeensyProtobuf(TeensyData* td) {
  if (td->has_rc_data) {
    RCData rcd = td->rc_data;
    Serial.printf(
        "RC Data:\n"
        "  Left Analog:  X=%d  Y=%d\n"
        "  Right Analog: X=%d  Y=%d\n"
        "  Switches: FL1=%d FL2=%d FR=%d\n"
        "  Top Switches: Left=%d Right=%d\n"
        "  Potentiometer: %d\n",
        rcd.left_analog_x, rcd.left_analog_y, rcd.right_analog_x, rcd.right_analog_y,
        rcd.front_left_switch1, rcd.front_left_switch2, rcd.front_right_switch, rcd.top_left_switch,
        rcd.top_right_switch, rcd.potentiometer);
  }

  if (td->has_windvane) {
    Serial.printf("Wind Vane: %d°\n", td->windvane.wind_angle);
  }

  if (td->has_water_sensors) {
    Serial.printf("Water Level: %d\n", td->water_sensors.water_level);
  }

  if (td->has_gps) {
    Serial.printf(
        "GPS:\n"
        "  Lat: %.6f\n"
        "  Lon: %.6f\n"
        "  Speed: %.2f m/s\n",
        td->gps.lat, td->gps.lon, td->gps.speed);
  }

  if (td->has_imu) {
    Serial.printf(
        "IMU:\n"
        "  Roll: %.2f°\n"
        "  Pitch: %.2f°\n"
        "  Yaw: %.2f°\n",
        td->imu.roll, td->imu.pitch, td->imu.yaw);
  }

  if (td->has_servos) {
    Serial.printf(
        "Servos:\n"
        "  Sail: %d\n"
        "  Jib: %d\n"
        "  Rudder: %d\n",
        td->servos.sail, td->servos.jib, td->servos.rudder);
  }

  if (td->has_camera_servos) {
    Serial.printf(
        "Camera Servos:\n"
        "  Yaw: %d\n"
        "  Pitch: %d\n",
        td->camera_servos.yaw, td->camera_servos.pitch);
  }
}
