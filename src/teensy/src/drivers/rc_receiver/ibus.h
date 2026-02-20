#pragma once
#include <Arduino.h>
#include <IBusBM.h>

#include "drivers/rc_receiver/rc_receiver.h"

/**
 * @brief IBus protocol receiver driver
 * IBus is a digital serial protocol used by our FlySky RC receiver
 */
class IBusReceiver : public RCReceiver {
 private:
  HardwareSerial* serial_port;
  IBusBM* ibus;
  static constexpr int RC_LOW = 1000;
  static constexpr int RC_HIGH = 2000;

 public:
  IBusReceiver(HardwareSerial* port);
  ~IBusReceiver();
  bool begin() override;
  bool readControllerState(RCData* controller) override;
};