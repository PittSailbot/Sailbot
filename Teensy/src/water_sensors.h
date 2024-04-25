#ifndef WATER_SENSORS_H
#define WATER_SENSORS_H
#include "teensy.pb.h"

extern void setupWaterSensors();
extern void readWaterSensors(WaterSensors*);

extern void setupPumps();
extern void enablePumps();
extern void disablePumps();

#endif