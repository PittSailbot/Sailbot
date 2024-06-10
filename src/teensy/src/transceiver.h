#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H
#include "teensy.pb.h"
#include <ArduinoJson.h>

extern void setupTransceiver();
extern bool readControllerState(JsonObject);

#endif