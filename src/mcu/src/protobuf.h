#ifndef PROTOBUF_H
#define PROTOBUF_H
#include <string>

#include "mcu.pb.h"
#include "pi.pb.h"

extern bool readProtobufFromPi(PiData*);
extern void writeProtobufToPi(TeensyData*);
extern void printTeensyProtobuf(TeensyData*);

#endif