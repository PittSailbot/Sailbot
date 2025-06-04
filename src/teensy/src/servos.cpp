// Reads and sets the servos for the rudder, main sail and jib
#include "servos.h"

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

#include "Adafruit_PWMServoDriver.h"
#include "teensy.h"

int last_sail_percent;
int last_jib_percent;
int last_rudder_angle;

bool readServos(Servos* servos) {
  servos->sail = last_sail_percent;
  servos->jib = last_jib_percent;
  servos->rudder = last_rudder_angle;
  return true;
}

#if SERVO_USE_GPIO
Servo SailServo;
Servo JibServo;
Servo RudderServo;

int setupServos() {
  Serial.println("I: Initializing servos");
  if (SAIL_SERVO != -1) {
    SailServo.attach(SAIL_SERVO, BILDA_MIN_ANGLE, BILDA_MAX_ANGLE);
    setSail(0);
    Serial.printf("I: Initialized SAIL servo on pin %d\n", SAIL_SERVO);
  } else {
    Serial.println("W: Skipping SAIL servo; no pin defined\n");
  }
  if (JIB_SERVO != -1) {
    JibServo.attach(JIB_SERVO, BILDA_MIN_ANGLE, BILDA_MAX_ANGLE);
    setJib(0);
    Serial.printf("I: Initialized JIB servo on pin %d\n", JIB_SERVO);
  } else {
    Serial.println("W: Skipping JIB servo; no pin defined");
  }
  if (RUDDER_SERVO != -1) {
    RudderServo.attach(RUDDER_SERVO, HITECH_MIN_ANGLE, HITECH_MAX_ANGLE);
    setRudder((HITECH_MAX_ANGLE - HITECH_MIN_ANGLE) / 2);
    Serial.printf("I: Initialized RUDDER servo on pin %d\n", RUDDER_SERVO);
  } else {
    Serial.println("W: Skipping RUDDER servo; no pin defined");
  }

  Serial.println("Servos initialized");
  return 0;
}

void setSail(int percentTensioned) {
  int pwm = map(percentTensioned, 0, 100, BILDA_MIN_PWM, BILDA_MAX_PWM);
  SailServo.writeMicroseconds(pwm);
  last_sail_percent = percentTensioned;
}

void setJib(int percentTensioned) {
  int pwm = map(percentTensioned, 0, 100, BILDA_MIN_PWM, BILDA_MAX_PWM);
  JibServo.writeMicroseconds(pwm);
  last_jib_percent = percentTensioned;
}

void setRudder(int angle) {
  int pwm = map(angle, HITECH_MIN_ANGLE, HITECH_MAX_ANGLE, HITECH_MIN_PWM, HITECH_MAX_PWM);
  RudderServo.writeMicroseconds(pwm);
  last_rudder_angle = angle;
}

#endif

#if SERVO_USE_I2C
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

int setupServos() {
  Serial.println("I: Initializing servos");
  driver.begin();
  driver.setPWMFreq(1600);

  if (SAIL_SERVO != -1) {
    setSail(0);
    Serial.printf("I: Initialized SAIL servo at address %x\n", SAIL_SERVO);
  } else {
    Serial.println("W: Skipping SAIL servo; no i2c address defined");
  }
  if (JIB_SERVO != -1) {
    setJib(0);
    Serial.printf("I: Initialized JIB servo at address %x\n", JIB_SERVO);
  } else {
    Serial.println("W: Skipping JIB servo; no i2c address defined");
  }
  if (RUDDER_SERVO != -1) {
    setRudder((HITECH_MAX_ANGLE - HITECH_MIN_ANGLE) / 2);
    Serial.printf("I: Initialized RUDDER servo at address %x\n", RUDDER_SERVO);
  } else {
    Serial.println("W: Skipping RUDDER servo; no i2c address defined");
  }

  Serial.println("I: Servos initialized");
  return 0;
}

void setSail(int percentTensioned) {
  int pwm = map(percentTensioned, 0, 100, BILDA_MIN_PWM, BILDA_MAX_PWM);
  driver.writeMicroseconds(SAIL_SERVO, pwm);
  last_sail_percent = percentTensioned;
}

void setJib(int percentTensioned) {
  int pwm = map(percentTensioned, 0, 100, BILDA_MIN_PWM, BILDA_MAX_PWM);
  driver.writeMicroseconds(JIB_SERVO, pwm);
  last_jib_percent = percentTensioned;
}

void setRudder(int angle) {
  int pwm = map(angle, HITECH_MIN_ANGLE, HITECH_MAX_ANGLE, HITECH_MIN_PWM, HITECH_MAX_PWM);
  driver.writeMicroseconds(RUDDER_SERVO, pwm);
  last_rudder_angle = angle;
}
#endif
