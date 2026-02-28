#pragma once
#include <Arduino.h>

#include "../../../lib/sbus/sbus.h"
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
  // bfs::SbusTx* sbus_tx; // Unused
  bfs::SbusData* data;
  static constexpr int RC_LOW = 150;
  static constexpr int RC_HIGH = 1811;

 public:
  SBusReceiver(HardwareSerial* port);
#if defined(ARDUINO_ARCH_RP2040)
  SBusReceiver(int8_t rxpin);  // Pin-based constructor for RP2040
#endif
  ~SBusReceiver();
  bool begin() override;
  bool readControllerState(RCData* controller) override;
};