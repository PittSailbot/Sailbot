#ifndef IMU_H
#define IMU_H
#include "teensy.pb.h"
#include <ArduinoJson.h>

#define FILTER_UPDATE_RATE_HZ 100

extern int setupIMU();
extern bool readIMU(JsonObject);
extern void updateIMU();


#endif