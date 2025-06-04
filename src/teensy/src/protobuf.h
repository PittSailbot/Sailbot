#ifndef PROTOBUF_H
#define PROTOBUF_H
#include <string>

#include "pi.pb.h"
#include "teensy.pb.h"

extern void readProtobufFromPi(PiData*);
extern void writeProtobufToPi(TeensyData*);

#endif