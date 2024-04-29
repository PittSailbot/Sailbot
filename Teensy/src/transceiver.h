#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H
#include "teensy.pb.h"

extern void setupTransceiver();
extern bool readControllerState(Controller*);

#endif