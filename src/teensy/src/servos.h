#pragma once
#include <Servo.h>

/**
 * @brief Base template class for servo driver implementations
 */
class ServoInterface {
 public:
  uint16_t min_pwm;
  uint16_t max_pwm;
  uint16_t min_angle;
  uint16_t max_angle;
  uint16_t angle = 0;

  virtual ~ServoInterface() = default;

  /**
   * @brief Set the servo to a specific angle
   * @param angle position to send servo to (in degrees); automatically constrained to min/max
   */
  virtual void write(int angle) = 0;

  /**
   * @brief Read the last set angle (in degrees)
   * @return angle (in degrees) that the angle was last set to. This assumes instantenous movement,
   * so actual angle may differ according to travel time
   */
  int read() {
    return angle;
  }
};

// Interface for driving servos directly over GPIO pins
class GPIOServoInterface : public ServoInterface {
 private:
  Servo servo;
  int pin_number;  // MUST be a PWM-capable pin

 public:
  GPIOServoInterface(int pin, uint16_t min_pwm, uint16_t max_pwm, uint16_t min_angle,
                     uint16_t max_angle);
  void write(int angle) override;
};

// Interface for driving servos over the Adafruit driver board
class I2CServoInterface : public ServoInterface {
 private:
  int channel;

 public:
  I2CServoInterface(int i2c_channel, uint16_t min_pwm, uint16_t max_pwm, uint16_t min_angle,
                    uint16_t max_angle);
  void write(int angle) override;
};