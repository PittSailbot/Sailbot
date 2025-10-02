// Driver implementations to reads the RC controller state from various receivers
#include "transceiver.h"

#include <Arduino.h>
#include <IBusBM.h>
#include <sbus.h>

// SBUS Receiver Implementation
SBusReceiver::SBusReceiver(HardwareSerial* port) : serial_port(port) {
  sbus_rx = new bfs::SbusRx(port);
  sbus_tx = new bfs::SbusTx(port);
  data = new bfs::SbusData();
}

SBusReceiver::~SBusReceiver() {
  delete sbus_rx;
  delete sbus_tx;
  delete data;
}

bool SBusReceiver::begin() {
  sbus_rx->Begin();
  sbus_tx->Begin();
  return true;
}

bool SBusReceiver::readControllerState(RCData* controller) {
  /* EXPECTED RC CONTROLLER FORMAT for FrSky X9 Lite receiver
  Max and min trim thresholds are within +-10. They do not effect the max/min value. They only
  offset the "center" value.
  - Down/Left reads ~172-180 (Converted to 0)
    - Front-facing switches are reversed and read 0 on up
  - Center reads ~980-1000 (Converted to 50)
  - Up/Right reads ~1795-1811 (Converted to 100)
  RC receiver still repeats the last controller state when it is off/connection lost. Could cause
  issues.

  Channels:
  0 - left_analog_y
  1 - right_analog_x
  2 - right_analog_y
  3 - left_analog_x
  4 - front_left_switch1
  5 - front_left_switch2
  6 - front_right_switch
  7 - top_left_switch
  8 - top_right_switch
  9 - potentiometer
  10-16 - UNUSED
  */

  if (sbus_rx->Read()) {
    *data = sbus_rx->data();

    // Data channels = 0 when controller is disconnected
    if (data->ch[0] == 0) {
      return false;
    }

    controller->left_analog_y = map(data->ch[0], RC_LOW, RC_HIGH, 0, 100);
    controller->right_analog_x = map(data->ch[1], RC_LOW, RC_HIGH, 0, 100);
    controller->right_analog_y = map(data->ch[2], RC_LOW, RC_HIGH, 0, 100);
    controller->left_analog_x = map(data->ch[3], RC_LOW, RC_HIGH, 0, 100);
    controller->front_left_switch1 = map(data->ch[4], RC_LOW, RC_HIGH, 0, 2);
    controller->front_left_switch2 = map(data->ch[5], RC_LOW, RC_HIGH, 0, 2);
    controller->front_right_switch = map(data->ch[6], RC_LOW, RC_HIGH, 0, 2);
    controller->top_left_switch = map(data->ch[7], RC_LOW, RC_HIGH, 0, 1);
    controller->top_right_switch = map(data->ch[8], RC_LOW, RC_HIGH, 0, 1);
    controller->potentiometer = map(data->ch[9], RC_LOW, RC_HIGH, 0, 100);

    return true;
  }
  return false;
}

// IBus Receiver Implementation
IBusReceiver::IBusReceiver(HardwareSerial* port) : serial_port(port) {
  ibus = new IBusBM();
}

IBusReceiver::~IBusReceiver() {
  delete ibus;
}

bool IBusReceiver::begin() {
  ibus->begin(*serial_port);
  return true;
}

bool IBusReceiver::readControllerState(RCData* controller) {
  // Data channels = 0 when controller is disconnected
  if (ibus->readChannel(0) == 0) {
    return false;
  }

  // Map IBus channels to controller data (channels are 1000-2000 range)
  controller->left_analog_y = map(ibus->readChannel(0), 1000, 2000, 0, 100);
  controller->right_analog_x = map(ibus->readChannel(1), 1000, 2000, 0, 100);
  controller->right_analog_y = map(ibus->readChannel(2), 1000, 2000, 0, 100);
  controller->left_analog_x = map(ibus->readChannel(3), 1000, 2000, 0, 100);
  controller->front_left_switch1 = map(ibus->readChannel(4), 1000, 2000, 0, 2);
  controller->front_left_switch2 = map(ibus->readChannel(5), 1000, 2000, 0, 2);
  controller->front_right_switch = map(ibus->readChannel(6), 1000, 2000, 0, 2);
  controller->top_left_switch = map(ibus->readChannel(7), 1000, 2000, 0, 1);
  controller->top_right_switch = map(ibus->readChannel(8), 1000, 2000, 0, 1);
  controller->potentiometer = map(ibus->readChannel(9), 1000, 2000, 0, 100);

  return true;
}