#ifndef CAMERA_SERVOS_H
#define CAMERA_SERVOS_H
#include "teensy.pb.h"

#define YAW_SERVO 1
#define PITCH_SERVO 0

#define SERVO_FREQ 50

#define GH_S37D_MIN_PWM 140
#define GH_S37D_MAX_PWM 600
#define GH_S37D_MIN_ANGLE 0
#define GH_S37D_MAX_ANGLE 180

#define YAW_MIN_ANGLE 60
#define YAW_MAX_ANGLE 155
#define PITCH_MIN_ANGLE 0
#define PITCH_MAX_ANGLE 180

bool setupCameraServos();
bool readCameraServos(CameraServos* servos);
void setCameraYaw(int angle);
void setCameraPitch(int angle);

#endif