#ifndef IMU_H
#define IMU_H
#include "teensy.pb.h"

#define FILTER_UPDATE_RATE_HZ 100

extern int setupIMU();
extern bool readIMU(IMU*);
extern void updateIMU();

#endif