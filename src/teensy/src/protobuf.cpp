#include "protobuf.h"

#include <ArduinoLog.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <sstream>

void readProtobufFromPi(PiData* pi_data) {
  if (Serial.available()) {
    uint8_t buffer[PI_PB_H_MAX_SIZE];
    pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_decode(&stream, PiData_fields, pi_data);
    Log.verboseln("Read protobuf from Pi: %s", PiDataToString(pi_data));

    if (!status) {
      Log.errorln("Failed to read protobuf from Pi: %s", stream.errmsg);
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
    Log.verboseln("Wrote protobuf to Pi: %s", TeensyDataToString(*teensy_data));
  } else {
    Log.errorln("Failed to write protobuf to Pi: %s", stream.errmsg);
  }
}

std::string PiDataToString(PiData* pi_data) {
  std::ostringstream oss;

  oss << "PiData {";

  // Camera Servos
  if (pi_data->has_cmd_yaw) {
    oss << " yaw: " << pi_data->cmd_yaw << "°";
  }
  if (pi_data->has_cmd_pitch) {
    oss << " pitch: " << pi_data->cmd_pitch << "°";
  }

  // Boat Servos
  if (pi_data->has_cmd_sail) {
    oss << " sail: " << pi_data->cmd_sail << "%";
  }
  if (pi_data->has_cmd_jib) {
    oss << " jib: " << pi_data->cmd_jib << "%";
  }
  if (pi_data->has_cmd_rudder) {
    oss << " rudder: " << pi_data->cmd_rudder << "%";
  }

  oss << " }";
  return oss.str();
}

std::string RCDataToString(RCData* rc) {
  std::ostringstream oss;
  oss << "RC{"
      << "LY:" << rc->left_analog_y << "%, "
      << "RX:" << rc->right_analog_x << "%, "
      << "RY:" << rc->right_analog_y << "%, "
      << "LX:" << rc->left_analog_x << "%, "
      << "FL1:" << rc->front_left_switch1 << ", "
      << "FL2:" << rc->front_left_switch2 << ", "
      << "FR:" << rc->front_right_switch << ", "
      << "TL:" << rc->top_left_switch << ", "
      << "TR:" << rc->top_right_switch << ", "
      << "POT:" << rc->potentiometer << "}";
  return oss.str();
}

std::string WindVaneToString(WindVane* wv) {
  return "Wind:" + std::to_string(wv->wind_angle) + "°";
}

std::string WaterSensorsToString(WaterSensors* ws) {
  return "Water:" + std::to_string(ws->water_level) + "mm";
}

std::string GPSToString(GPSData* gps) {
  std::ostringstream oss;
  oss.precision(6);
  oss << "GPS{"
      << "lat:" << std::fixed << gps->lat << ", "
      << "lon:" << std::fixed << gps->lon << ", "
      << "spd:" << gps->speed << "m/s}";
  return oss.str();
}

std::string IMUToString(IMU* imu) {
  std::ostringstream oss;
  oss.precision(2);
  oss << "IMU{"
      << "roll:" << std::fixed << imu->roll << "°, "
      << "pitch:" << imu->pitch << "°, "
      << "yaw:" << imu->yaw << "°}";
  return oss.str();
}

std::string ServosToString(Servos* s) {
  return "Servos{sail:" + std::to_string(s->sail) + "%, jib:" + std::to_string(s->jib) +
         "%, rudder:" + std::to_string(s->rudder) + "%}";
}

std::string CameraServosToString(CameraServos* cs) {
  return "Cam{yaw:" + std::to_string(cs->yaw) + "°, pitch:" + std::to_string(cs->pitch) + "°}";
}

std::string TeensyDataToString(TeensyData td) {
  std::ostringstream oss;
  oss << "TeensyData[";

  if (&td.has_rc_data) oss << RCDataToString(&td.rc_data) << " ";
  if (&td.has_windvane) oss << WindVaneToString(&td.windvane) << " ";
  if (&td.has_water_sensors) oss << WaterSensorsToString(&td.water_sensors) << " ";
  if (&td.has_gps) oss << GPSToString(&td.gps) << " ";
  if (&td.has_imu) oss << IMUToString(&td.imu) << " ";
  if (&td.has_servos) oss << ServosToString(&td.servos) << " ";
  if (&td.has_camera_servos) oss << CameraServosToString(&td.camera_servos);

  return oss.str();
}
