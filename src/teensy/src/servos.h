#ifndef SERVOS_H
#define SERVOS_H
#include "teensy.pb.h"

// PWM ranges that the servos will respond to
// Update if any of our servos are replaced with a different model
#define BILDA_MIN_PWM 500
#define BILDA_MAX_PWM 2500
#define HITECH_MIN_PWM 870
#define HITECH_MAX_PWM 2320

// Max travel range in degrees that the servo can travel
#define BILDA_MIN_ANGLE 0
#define BILDA_MAX_ANGLE 1800
#define HITECH_MIN_ANGLE 0
#define HITECH_MAX_ANGLE 146

extern int setupServos();
extern void setSail(int);
extern void setJib(int);
extern void setRudder(int);
extern bool readServos(Servos*);

#endif