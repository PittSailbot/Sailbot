// #include "camera_servos.h"

// #include <Adafruit_PWMServoDriver.h>

// #include "teensy.h"
// #include "teensy.pb.h"

// int last_camera_yaw;
// int last_camera_pitch;
// bool cam_initialized = false;

// Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver(CAMERA_SERVOS_I2C);

// bool readCameraServos(CameraServos* servos) {
//   if (!cam_initialized) {
//     return false;
//   }
//   servos->yaw = last_camera_yaw;
//   servos->pitch = last_camera_pitch;
//   return true;
// }

// bool setupCameraServos() {
//   cam_initialized = driver.begin();
//   if (cam_initialized) {
//     driver.setOscillatorFrequency(27000000);
//     driver.setPWMFreq(SERVO_FREQ);
//     setCameraYaw(90);
//     setCameraPitch(90);

//     Serial.println("Camera Servos initialized");
//   } else {
//     Serial.println("Failed to initialize Camera Servos");
//   }

//   return cam_initialized;
// }

// void setCameraYaw(int angle) {
//   angle = constrain(angle, YAW_MIN_ANGLE, YAW_MAX_ANGLE);
//   int pwm = map(angle, GH_S37D_MIN_ANGLE, GH_S37D_MAX_ANGLE, GH_S37D_MIN_PWM, GH_S37D_MAX_PWM);
//   driver.setPWM(YAW_SERVO, 0, pwm);
//   last_camera_yaw = angle;
// }

// void setCameraPitch(int angle) {
//   angle = constrain(angle, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
//   int pwm = map(angle, GH_S37D_MIN_ANGLE, GH_S37D_MAX_ANGLE, GH_S37D_MIN_PWM, GH_S37D_MAX_PWM);
//   driver.setPWM(PITCH_SERVO, 0, pwm);
//   last_camera_pitch = angle;
// }
