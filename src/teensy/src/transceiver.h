#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H
#include "teensy.pb.h"

#define SWITCH_UP 1
#define SWITCH_DOWN 0
#define TRI_SWITCH_UP 0
#define TRI_SWITCH_MID 1
#define TRI_SWITCH_DOWN 2

extern void setupTransceiver();
extern bool readControllerState(RCData*);

#endif