#ifndef WATER_SENSORS_H
#define WATER_SENSORS_H
#include "teensy.pb.h"

extern void setupWaterSensors();
extern bool readWaterSensors(WaterSensors*);

extern void setupPumps();
extern void enablePumps();
extern void disablePumps();
extern void pumpOnSensors();

#endif