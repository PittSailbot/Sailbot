#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <elapsedMillis.h>

#include "hal/components.h"

/**
 * @brief Base template class for servo driver implementations
 */
class ServoInterface {
 public:
  uint16_t min_pwm;
  uint16_t max_pwm;
  uint16_t min_angle = 0;
  uint16_t max_angle;
  uint16_t angle = 0;
  uint16_t max_deg_s;
  uint16_t last_angle = 65535;
  // Speed limiting servo
  uint16_t desired_angle = 0;
  float speed_credit = 0;
  elapsedMillis dt = 0; // time since last written to

  virtual ~ServoInterface() = default;


  /**
   * @brief Set the servo to a specific angle
   * @param angle position to send servo to (in degrees); automatically constrained to min/max
   */
  virtual void write(int angle) = 0;

  /**
   * @brief Read the current commanded angle (in degrees)
   * @return angle (in degrees) that the servo is currently commanded to move to
   */
  int read() {
    return angle;
  }

  /**
   * @brief Set servo to center position
   */
  void center() {
    this->write((this->max_angle - this->min_angle) / 2);
  }

  /**
   * @brief Set the servo to a percent of its maximum travel range
   */
  void writePercent(int percent) {
    percent = constrain(percent, 0, 100);

    int angle = map(percent, 0, 100, min_angle, max_angle);

    this->write(angle);
  }

  /**
   * @brief Clamp servo rate of change to max allowed speed
   * Side effect: this relies on servo.write() being called frequently, otherwise servo may never achieve set angle
   * @return clamped servo angle, either the requested value if < speed or the max allowed change / s
   */
  int clampServoSpeed(int requestedAngle) {
    int delta_angle = abs(requestedAngle - this->angle);
    this->speed_credit += this->max_deg_s * (this->dt / 1000.0f);

    int max_delta = static_cast<int>(this->speed_credit);

    if (delta_angle > max_delta) {
      if (requestedAngle > this->angle) {
        this->speed_credit -= max_delta;
        return this->angle + max_delta;
      } else {
        this->speed_credit -= max_delta;
        return this->angle - max_delta;
      }
    } else {
      this->speed_credit -= delta_angle;
      return requestedAngle;
    }
  }
};

// Interface for driving servos directly over GPIO pins
class GPIOServoInterface : public ServoInterface {
 private:
  Servo servo;
  int pin_number;  // MUST be a PWM-capable pin

 public:
  GPIOServoInterface(int pin, uint16_t min_pwm, uint16_t max_pwm, uint16_t max_angle, uint16_t max_deg_s);
  GPIOServoInterface(int pin, ServoSpecData spec);
  void write(int angle) override;
};

// Interface for driving servos over the Adafruit driver board
class I2CServoInterface : public ServoInterface {
 private:
  int channel;

 public:
  I2CServoInterface(int i2c_channel, uint16_t min_pwm, uint16_t max_pwm, uint16_t max_angle, uint16_t max_deg_s);
  I2CServoInterface(int i2c_channel, ServoSpecData spec);
  void write(int angle) override;
};