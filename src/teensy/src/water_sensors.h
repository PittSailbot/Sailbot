#ifndef WATER_SENSORS_H
#define WATER_SENSORS_H
#include "teensy.pb.h"

// Reading levels from the capacitve sensors (low = dry, high = wet)
#define WATER_LEVEL_LOW 200
#define WATER_LEVEL_HIGH 2000

// required water level reading (0-100) for bilge pump to enable
#define ENABLE_PUMP_THRESHOLD 30  // TODO: Test in real-world conditions

extern void setupWaterSensors();
extern bool readWaterSensors(WaterSensors*);

extern void setupPumps();
extern void pumpIfWaterDetected();
extern void enablePumps();
extern void disablePumps();

#endif