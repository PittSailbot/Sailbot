#ifndef WATER_SENSORS_H
#define WATER_SENSORS_H
#include "teensy.pb.h"

// Reading levels from the capacitve sensors (low = dry, high = fully submerged)
// When submerged, readings jump from ~350-650 nearly instantly then slowly climb to 700 until it is
// fully submerged (Pitt tap water) Air ~320 Water (any) ~500 Water end of tip ~670 Fully submerged
// ~700 Touching the sensor maxes it at 1016
#define WATER_LEVEL_LOW 650
#define WATER_LEVEL_HIGH 700

// required water level reading (0-100) for bilge pump to enable
#define ENABLE_PUMP_THRESHOLD 20

extern void setupWaterSensors();
extern bool readWaterSensors(WaterSensors*);

extern void setupPumps();
extern void pumpIfWaterDetected();
extern void enablePumps();
extern void disablePumps();

#endif