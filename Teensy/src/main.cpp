// Main program running on the Teensy
// Reads and controls most of the sensors on the boat and interfaces with the Pi via protobuf
#include <Arduino.h>
#include <sbus.h>
#include <pb_encode.h>
#include "teensy.pb.h"
#include "transceiver.h"
#include "windvane.h"


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  setupTransceiver();
  setupWindvane();
}

void loop () {
  Data pi_data = Data_init_default;

  readControllerState(&pi_data.controller);
  pi_data.windvane.wind_angle = readWindvane();

  uint8_t buffer[TEENSY_PB_H_MAX_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_encode(&stream, Data_fields, &pi_data);

  for (uint8_t byte : buffer) {
    Serial.print(byte);
  }
  Serial.println();

  delay(100);
}
