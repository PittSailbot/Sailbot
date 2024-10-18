#ifndef GPS_H
#define GPS_H
#include "teensy.pb.h"

extern void setupGPS();
extern bool readGPS(GPSData*);

#endif