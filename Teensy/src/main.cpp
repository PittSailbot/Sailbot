// Main program running on the Teensy
// Reads and controls most of the sensors on the boat and interfaces with the Pi via protobuf
#include <Arduino.h>
#include <sbus.h>
#include <pb_encode.h>
#include "teensy.h"
#include "teensy.pb.h"
#include "transceiver.h"
#include "windvane.h"
#include "water_sensors.h"


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  setupTransceiver();
  setupWindVane();
  setupWaterSensors();
  setupPumps();
}

void loop () {
  Data pi_data = Data_init_default;

  readControllerState(&pi_data.controller);
  readWindVane(&pi_data.windvane);
  readWaterSensors(&pi_data.water_sensors);

  /*if (pi_data.water_sensors.sensor1_is_wet || pi_data.water_sensors.sensor2_is_wet || pi_data.water_sensors.sensor3_is_wet) {
    enablePumps()
  }*/

  uint8_t buffer[TEENSY_PB_H_MAX_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_encode(&stream, Data_fields, &pi_data);

  if (status) {
    for (uint8_t byte : buffer) {
      Serial.print(byte);
    }
    Serial.println();
  } else {
    Serial.println(stream.errmsg);
  }

  delay(100);
}
