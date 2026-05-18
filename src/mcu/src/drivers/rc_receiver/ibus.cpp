// Driver implementations to reads the RC controller state from various receivers
#include "drivers/rc_receiver/ibus.h"

#include <Arduino.h>
#include <IBusBM.h>

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

  // Map IBus channels to controller data (channels are RC_LOW-RC_HIGH range)
  controller->left_analog_y = map(ibus->readChannel(0), RC_LOW, RC_HIGH, 0, 100);
  controller->right_analog_x = map(ibus->readChannel(1), RC_LOW, RC_HIGH, 0, 100);
  controller->right_analog_y = map(ibus->readChannel(2), RC_LOW, RC_HIGH, 0, 100);
  controller->left_analog_x = map(ibus->readChannel(3), RC_LOW, RC_HIGH, 0, 100);
  controller->front_left_switch1 = map(ibus->readChannel(4), RC_LOW, RC_HIGH, 0, 2);
  controller->front_left_switch2 = map(ibus->readChannel(5), RC_LOW, RC_HIGH, 0, 2);
  controller->front_right_switch = map(ibus->readChannel(6), RC_LOW, RC_HIGH, 0, 2);
  controller->top_left_switch = map(ibus->readChannel(7), RC_LOW, RC_HIGH, 0, 1);
  controller->top_right_switch = map(ibus->readChannel(8), RC_LOW, RC_HIGH, 0, 1);
  controller->potentiometer = map(ibus->readChannel(9), RC_LOW, RC_HIGH, 0, 100);

  return true;
}