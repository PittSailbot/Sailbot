#pragma once
#include <Arduino.h>
#include <IBusBM.h>
#include <sbus.h>

#include "teensy.pb.h"

// Macro assumes switch up/down is the same across all receivers which might not be true)
#define SWITCH_UP 1
#define SWITCH_DOWN 0
#define TRI_SWITCH_UP 0
#define TRI_SWITCH_MID 1
#define TRI_SWITCH_DOWN 2

/**
 * @brief Base template class for RC receiver implementations
 */
class RCReceiver {
 public:
  virtual ~RCReceiver() = default;

  /**
   * @brief Initialize the receiver
   * @return true on success, false on fail
   */
  virtual bool begin() = 0;

  /**
   * @brief Read the current controller state
   * @param controller pointer to RCData struct to populate with controller values
   * @return true if valid data was read, false otherwise
   */
  virtual bool readControllerState(RCData* controller) = 0;
};

// SBUS Receiver Implementation
class SBusReceiver : public RCReceiver {
 private:
  HardwareSerial* serial_port;
  bfs::SbusRx* sbus_rx;
  bfs::SbusTx* sbus_tx;
  bfs::SbusData* data;
  static const int RC_LOW = 150;
  static const int RC_HIGH = 1811;

 public:
  SBusReceiver(HardwareSerial* port);
  ~SBusReceiver();
  bool begin() override;
  bool readControllerState(RCData* controller) override;
};

// IBus Receiver Implementation
class IBusReceiver : public RCReceiver {
 private:
  HardwareSerial* serial_port;
  IBusBM* ibus;
  static const int RC_LOW = 151;
  static const int RC_HIGH = 2000;

 public:
  IBusReceiver(HardwareSerial* port);
  ~IBusReceiver();
  bool begin() override;
  bool readControllerState(RCData* controller) override;
};