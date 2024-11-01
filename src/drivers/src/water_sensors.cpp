// Reads if water is detected by any of the sensors and controls the pumps
#include "water_sensors.h"
#include "teensy.h"

void setupWaterSensors() {
  if (WATER_SENSOR1_INSTALLED){
    pinMode (WATER_SENSOR1,INPUT_PULLDOWN);
  }
  if (WATER_SENSOR2_INSTALLED){
    pinMode (WATER_SENSOR2,INPUT_PULLDOWN);
  }
  if (WATER_SENSOR3_INSTALLED){
    pinMode (WATER_SENSOR3,INPUT_PULLDOWN);
  }
  
  

  Serial.println("Started Water Sensors");
}

void setupPumps() {
  pinMode (PUMP1,OUTPUT);

  Serial.println("Started Pumps");
}

bool readWaterSensors(WaterSensors* water_sensors) {

  if (WATER_SENSOR1_INSTALLED){
    water_sensors->sensor1_is_wet = digitalRead(WATER_SENSOR1);
    water_sensors->has_sensor1_is_wet = true;
  }
  if (WATER_SENSOR2_INSTALLED){
    water_sensors->sensor2_is_wet = digitalRead(WATER_SENSOR2);
    water_sensors->has_sensor2_is_wet = true;
  }
  if (WATER_SENSOR3_INSTALLED){
    water_sensors->sensor3_is_wet = digitalRead(WATER_SENSOR3);
    water_sensors->has_sensor3_is_wet = true;
  }
  
  return WATER_SENSOR1_INSTALLED || WATER_SENSOR2_INSTALLED || WATER_SENSOR3_INSTALLED;
}

void pumpOnSensors(){
  digitalWrite(PUMP1, digitalRead(WATER_SENSOR1));
  digitalWrite(PUMP2, digitalRead(WATER_SENSOR2));
  digitalWrite(PUMP3, digitalRead(WATER_SENSOR3));
}

void enablePumps() {
  digitalWrite(PUMP1, HIGH);
  digitalWrite(PUMP2, HIGH);
  digitalWrite(PUMP3, HIGH);
}

void disablePumps() {
  digitalWrite(PUMP1, LOW);
  digitalWrite(PUMP2, LOW);
  digitalWrite(PUMP3, LOW);
}