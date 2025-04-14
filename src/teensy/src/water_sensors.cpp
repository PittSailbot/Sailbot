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
    Serial.println("ERROR! seesaw not found");
    return;
  }

  Serial.println("Started Water Sensors");
}

bool readWaterSensors(WaterSensors* water_sensors) {
  if (initialized) {
    // float tempC = ss.getTemp();
    uint16_t capacitance = ss.touchRead(0);
    water_level = map(capacitance, WATER_LEVEL_LOW, WATER_LEVEL_HIGH, 0, 100);
    water_sensors->water_level = water_level;
  }

  return initialized;
}

void setupPumps() {
  if (PUMP1 != -1) {
    pinMode(PUMP1, OUTPUT);
    Serial.println("Started Pumps");
  } else {
    Serial.println("Skipping pumps; no pin defined");
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
  Serial.println("Enabling pumps");
}

void disablePumps() {
  digitalWrite(PUMP1, LOW);
  Serial.println("Disabling pumps");
}