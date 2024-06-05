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
#include "receiveCmds.h"
#include <Wire.h>
#include <IntervalTimer.h>
#include <ArduinoJson.h>

int pwm_val = 0;
int pwm_peak = 150;

IntervalTimer filterTimer;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  // while (!Serial) {}
  setupTransceiver();
  setupWindVane();
  // setupGPS();
  setupIMU();
  if (!filterTimer.begin(updateIMU, int(1000000 / FILTER_UPDATE_RATE_HZ))) {
    Serial.println("Failed to start filter timer");
  }
  setupWaterSensors();
  //setupPumps();
  setupReceiver();

  Serial.println("Initialized Teensy");

  
}

void loop () {
  const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(2) + JSON_ARRAY_SIZE(2) + 100;
  DynamicJsonDocument doc(capacity);
  JsonObject rc_data = doc.createNestedObject("rc_data");
  Data pi_data = Data_init_default;
  pi_data.has_rc_data = readControllerState(rc_data);
  JsonObject windvane = doc.createNestedObject("windvane");
  pi_data.has_windvane = readWindVane(windvane);
  pi_data.has_gps = false; //readGPS(&pi_data.gps);
  JsonObject imu = doc.createNestedObject("imu");
  pi_data.has_imu = readIMU(imu);
  JsonObject water_sensors = doc.createNestedObject("water_sensors");
  pi_data.has_water_sensors = readWaterSensors(water_sensors);
  pumpOnSensors();

  pwm_val = (pwm_val + 10) % pwm_peak;
  analogWrite(13, pwm_val);
  if (pi_data.has_imu && pi_data.has_rc_data && pi_data.has_windvane){ // steady blue light if all usb sensors are working
    digitalWrite(20, HIGH);
  }
  else if (pi_data.has_imu || pi_data.has_rc_data || pi_data.has_windvane){ // blinking blue light if some are working
    digitalWrite(20, pwm_val > (pwm_peak/2));
  }
  else{
    digitalWrite(20, LOW);
  }

  uint8_t buffer[TEENSY_PB_H_MAX_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_encode(&stream, Data_fields, &pi_data);
  String jsonString;
  serializeJson(doc, jsonString);

  // Print the JSON string
  Serial.println(jsonString);
  // if (status) {
  //   Serial.write(buffer, stream.bytes_written);
  //   Serial.println();
  // } else {
  //   Serial.print("ERROR: ");
  //   Serial.println(stream.errmsg);
  // }

  delay(100);
}
