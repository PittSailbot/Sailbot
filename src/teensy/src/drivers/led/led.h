#pragma once
#include <jled.h>

class Status_Led : public JLed {
 public:
  enum class Mode {
    OK,             // ON | All sensors & microcontroller are working as expected
    RC_OFF,         // Steady slow blink | RC controller reads but not connected
    IMU_CAL,        // Double moderate blink | IMU needs calibration
    WINDVANE_FAIL,  // Triple fast blink | Windvane sensor is failing to read
    INIT_ERROR,     // Breathe | Some sensor/component failed to init (likely wiring)
  };

  Status_Led(uint8_t pin) : JLed(pin), _displayed(Mode::OK), _flags(0) {
    _applyPattern(Mode::OK);
  }

  // Mark a mode as active
  void raise(Mode mode) {
    _flags |= _bit(mode);
    _refresh();
  }

  // Mark a mode as resolved
  void lower(Mode mode) {
    _flags &= ~_bit(mode);
    _refresh();
  }

 private:
  Mode _displayed;
  uint8_t _flags;

  static uint8_t _bit(Mode mode) {
    return 1 << static_cast<uint8_t>(mode);
  }

  void _refresh() {
    // If multiple statuses are active, set to the highest priority one
    // Priority: INIT_ERROR > WINDVANE_FAIL > IMU_CAL > RC_OFF > OK
    Mode active = Mode::OK;
    if (_flags & _bit(Mode::INIT_ERROR))
      active = Mode::INIT_ERROR;
    else if (_flags & _bit(Mode::WINDVANE_FAIL))
      active = Mode::WINDVANE_FAIL;
    else if (_flags & _bit(Mode::IMU_CAL))
      active = Mode::IMU_CAL;
    else if (_flags & _bit(Mode::RC_OFF))
      active = Mode::RC_OFF;

    // Do nothing if pattern is unchanged so that animation doesn't reset
    if (active == _displayed) return;
    _displayed = active;
    _applyPattern(active);
  }

  void _applyPattern(Mode mode) {
    switch (mode) {
      case Mode::OK:
        On();
        break;
      case Mode::RC_OFF:
        Blink(500, 500).Forever();
        break;
      case Mode::IMU_CAL:
        Blink(200, 200).Repeat(2).DelayAfter(600).Forever();
        break;
      case Mode::WINDVANE_FAIL:
        Blink(100, 100).Repeat(3).DelayAfter(800).Forever();
        break;
      case Mode::INIT_ERROR:
        Breathe(1000).Forever();
    }
  }
};
