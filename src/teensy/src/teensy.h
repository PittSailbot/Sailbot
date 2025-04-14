// Wired connections between the PCB and the Teensy
// Teensy pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
// If any pinouts to the Teensy are changed, update them here:

// PCB -> Teensy
#define AUXFET1 38
#define AUXFET2 2
#define AUXFET3 29
#define AUXFET4 28

// Teensy -> Sensors/Controls
#define SERVO_USE_I2C false
#define SERVO_USE_GPIO !SERVO_USE_I2C

#if SERVO_USE_GPIO
#define SAIL_SERVO -1    // TODO: Set to servo control pin
#define JIB_SERVO -1     // TODO: Set to servo control pin
#define RUDDER_SERVO -1  // TODO: Set to servo control pin
#endif
#if SERVO_USE_I2C  // Are the servos connected to an adafruit PWM servo driver board?
// https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino
// I2C address for the servo driver board is 0x40 (base) + number on terminal block
#define SAIL_SERVO 0x41
#define JIB_SERVO 0x42
#define RUDDER_SERVO 0x40
#endif

#define WATER_SENSOR1 0x36
#define PUMP1 AUXFET4

#define WINDVANE_ENCODER_A 34
#define WINDVANE_ENCODER_B 35

#define SLAVE_ADDRESS 0x10

// the first byte in an i2c command, to indicate the intention of the message
// absolute movements for the camera
#define CAM_H_MOVE_ABS_CMD 0
#define CAM_V_MOVE_ABS_CMD 1
#define CAM_MOVE_ABS_CMD 2
