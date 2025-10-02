// Driver implementations to reads and sets the servos for the rudder, main sail and jib
#include "servos.h"

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

#include "Adafruit_PWMServoDriver.h"

// GPIOServoInterface Implementation
GPIOServoInterface::GPIOServoInterface(int pin, uint16_t min_pwm, uint16_t max_pwm,
                                       uint16_t min_angle, uint16_t max_angle) {
  this->min_pwm = min_pwm;
  this->max_pwm = max_pwm;
  this->min_angle = min_angle;
  this->max_angle = max_angle;
  this->pin_number = pin;
  this->angle = 0;
  servo.attach(pin);
}

void GPIOServoInterface::write(int angle) {
  angle = constrain(angle, min_angle, max_angle);

  int pwm = map(angle, min_angle, max_angle, min_pwm, max_pwm);
  servo.writeMicroseconds(pwm);

  this->angle = angle;
}

// I2CServoInterface Implementation
extern Adafruit_PWMServoDriver driver;  // Use existing driver from servo code

I2CServoInterface::I2CServoInterface(int i2c_channel, uint16_t min_pwm, uint16_t max_pwm,
                                     uint16_t min_angle, uint16_t max_angle) {
  this->channel = i2c_channel;
  this->min_pwm = min_pwm;
  this->max_pwm = max_pwm;
  this->min_angle = min_angle;
  this->max_angle = max_angle;
  this->angle = 0;
}

void I2CServoInterface::write(int angle) {
  angle = constrain(angle, min_angle, max_angle);

  int pwm = map(angle, min_angle, max_angle, min_pwm, max_pwm);
  driver.writeMicroseconds(channel, pwm);

  this->angle = angle;
}
