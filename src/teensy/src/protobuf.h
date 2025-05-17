#ifndef PROTOBUF_H
#define PROTOBUF_H
#include <string>

#include "pi.pb.h"
#include "teensy.pb.h"

extern void readProtobufFromPi(PiData*);
extern void writeProtobufToPi(TeensyData*);

extern std::string PiDataToString(PiData*);

extern std::string RCDataToString(RCData*);
extern std::string WindVaneToString(WindVane*);
extern std::string WaterSensorsToString(WaterSensors*);
extern std::string GPSToString(GPSData*);
extern std::string IMUToString(IMU*);
extern std::string ServosToString(Servos*);
extern std::string CameraServosToString(CameraServos*);
extern std::string TeensyDataToString(TeensyData);

#endif