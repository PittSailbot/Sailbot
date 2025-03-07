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
  Data pi_data = Data_init_default;
  pi_data.has_rc_data = readControllerState(&pi_data.rc_data);
  pi_data.has_windvane = readWindVane(&pi_data.windvane);
  pi_data.has_gps = false; // readGPS(&pi_data.gps);
  pi_data.has_imu = readIMU(&pi_data.imu);
  pi_data.has_water_sensors = readWaterSensors(&pi_data.water_sensors);
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

  if (status) {
    Serial.write(buffer, stream.bytes_written);
    Serial.println();
  } else {
    Serial.print("ERROR: ");
    Serial.println(stream.errmsg);
  }

  delay(100);
}
