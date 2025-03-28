// Reads and sets the servos for the rudder, main sail and jib
#include <Arduino.h>
#include <Servo.h>
#include "servos.h"
#include "teensy.h"

Servo SailServo;
Servo JibServo;
Servo RudderServo;

int setupServos() {
  Serial.println("Initializing servos");
  if (SAIL_SERVO != NULL) {
    SailServo.attach(SAIL_SERVO);
    setSail(90);
  } else {
    Serial.println("Skipping SAIL servo; no pin defined");
  }
  if (JIB_SERVO != NULL) {
    JibServo.attach(JIB_SERVO);
    setJib(90);
  } else {
    Serial.println("Skipping JIB servo; no pin defined");
  }
  if (RUDDER_SERVO != NULL) {
    RudderServo.attach(RUDDER_SERVO);
    setRudder(90);
  } else {
    Serial.println("Skipping RUDDER servo; no pin defined");
  }
    
  Serial.println("Servos initialized");
  return 0;
}

bool setSail(int angle) {
  // int angle = map(reading, 0, 1023, 0, 180);
  SailServo.write(angle);
  return true;
}

bool setJib(int angle) {
  return false;
}

bool setRudder(int angle) {
  return false;
}
