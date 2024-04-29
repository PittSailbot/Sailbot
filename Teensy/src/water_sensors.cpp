// Reads if water is detected by any of the sensors and controls the pumps
#include "water_sensors.h"
#include "teensy.h"
#include <Arduino.h>

void setupWaterSensors() {
  pinMode (WATER_SENSOR1,INPUT_PULLDOWN);
  pinMode (WATER_SENSOR2,INPUT_PULLDOWN);
  pinMode (WATER_SENSOR3,INPUT_PULLDOWN);

  Serial.println("Started Water Sensors");
}

void setupPumps() {
  pinMode (PUMP1,OUTPUT);

  Serial.println("Started Pumps");
}

void readWaterSensors(WaterSensors* water_sensors) {
  water_sensors->sensor1_is_wet = digitalRead(WATER_SENSOR1);
  water_sensors->sensor2_is_wet = digitalRead(WATER_SENSOR2);
  water_sensors->sensor3_is_wet = digitalRead(WATER_SENSOR3);
}

void enablePumps() {
  digitalWrite(PUMP1, HIGH);
}

void disablePumps() {
  digitalWrite(PUMP1, LOW);
}