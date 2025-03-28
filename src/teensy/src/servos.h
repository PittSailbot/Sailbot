#ifndef SERVOS_H
#define SERVOS_H
#include "teensy.pb.h"

extern int setupServos();
extern bool setSail(int);
extern bool setJib(int);
extern bool setRudder(int);
extern bool readServos(Servos*);

#endif