// Main program running on the Teensy
// Reads and controls most of the sensors on the boat and interfaces with the Pi via protobuf
#include <Arduino.h>
#include <sbus.h>
#include <pb_encode.h>
#include "teensy.h"
#include "teensy.pb.h"
#include "transceiver.h"
#include "windvane.h"
#include "gps.h"
#include "imu.h"
#include "water_sensors.h"


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  setupTransceiver();
  // TODO: one of these is reading the Teensy's serialized messages (GPU?) and screwing with transceiver.py (enable one by one)
  //setupWindVane();
  // setupGPS();
  // setupIMU();
  //setupWaterSensors();
  //setupPumps();

  Serial.println("Initialized Teensy");
}

void loop () {
  Data pi_data = Data_init_default;

  pi_data.has_controller = readControllerState(&pi_data.controller);
  pi_data.controller.left_analog_x = 50;
  pi_data.has_controller = true;
  //readWindVane(&pi_data.windvane);
  //pi_data.has_windvane = true;
  //readGPS(&pi_data.gps);
  //pi_data.has_gps = true;
  //readIMU(&pi_data.imu);
  //pi_data.has_imu = true;
  //readWaterSensors(&pi_data.water_sensors);
  //pi_data.has_water_sensors = true;

  /*if (pi_data.water_sensors.sensor1_is_wet || pi_data.water_sensors.sensor2_is_wet || pi_data.water_sensors.sensor3_is_wet) {
    enablePumps()
  }*/

  uint8_t buffer[TEENSY_PB_H_MAX_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_encode(&stream, Data_fields, &pi_data);

  if (status) {
    Serial.println(stream.bytes_written);
    Serial.write(buffer, stream.bytes_written);
    /*for (uint8_t byte : buffer) {
      Serial.print(byte);
    }*/
    //Serial.println();
  } else {
    Serial.print("ERROR: ");
    Serial.println(stream.errmsg);
  }

  delay(100);
}
