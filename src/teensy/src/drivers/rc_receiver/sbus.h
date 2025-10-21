#pragma once
#include <Arduino.h>
#include <sbus.h>

#include "drivers/rc_receiver/rc_receiver.h"

/**
 * @brief SBus protocol receiver driver
 * SBus is a digital serial protocol used by our FrSky receiver
 * SBUS is basically just an inverted UART protocol
 */
class SBusReceiver : public RCReceiver {
 private:
  HardwareSerial* serial_port;
  bfs::SbusRx* sbus_rx;
  bfs::SbusTx* sbus_tx;
  bfs::SbusData* data;
  static constexpr int RC_LOW = 150;
  static constexpr int RC_HIGH = 1811;

 public:
  SBusReceiver(HardwareSerial* port);
  ~SBusReceiver();
  bool begin() override;
  bool readControllerState(RCData* controller) override;
};