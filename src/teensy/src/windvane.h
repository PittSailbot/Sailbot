#ifndef WINDVANE_H
#define WINDVANE_H
#include "teensy.pb.h"
#include <ArduinoJson.h>

extern void setupWindVane();
extern bool readWindVane(JsonObject);

#endif
