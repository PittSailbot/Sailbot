// Reads the relative water level and controls the pumps
#include "water_sensors.h"

#include <Adafruit_seesaw.h>
#include <Arduino.h>

#include "teensy.h"

Adafruit_seesaw ss;
bool initialized = false;
int water_level;

void setupWaterSensors() {
  initialized = ss.begin(WATER_SENSOR1);
  if (!initialized) {
    initialized = ss.begin(WATER_SENSOR2);
    if (!initialized) {
      Serial.println("W: Water level sensor not found");
      return;
    }
  }

  Serial.println("I: Started Water Sensors");
}

bool readWaterSensors(WaterSensors* water_sensors) {
  if (initialized) {
    // float tempC = ss.getTemp();
    uint16_t capacitance = ss.touchRead(0);  // TODO: filter
    if (capacitance > 1000) {
      // Someone is touching the sensor or the wire is poorly connected
      // (water shouldn't usually have this high capacitance)
      capacitance = 0;
    }
    water_level = constrain(capacitance, WATER_LEVEL_LOW, WATER_LEVEL_HIGH);
    water_level = map(capacitance, WATER_LEVEL_LOW, WATER_LEVEL_HIGH, 0, 100);
    water_sensors->water_level = water_level;
  }

  return initialized;
}

void setupPumps() {
  if (PUMP1 != -1) {
    pinMode(PUMP1, OUTPUT);
    Serial.println("I: Started Pumps");
  } else {
    Serial.println("W: Skipping pumps; no pin defined");
  }
}

void pumpIfWaterDetected() {
  if (!initialized || PUMP1 != -1) {
    return;
  }

  if (water_level >= ENABLE_PUMP_THRESHOLD) {
    enablePumps();
  }
}

void enablePumps() {
  digitalWrite(PUMP1, HIGH);
  Serial.println("I: Enabling pumps");
}

void disablePumps() {
  digitalWrite(PUMP1, LOW);
  Serial.println("I: Disabling pumps");
}