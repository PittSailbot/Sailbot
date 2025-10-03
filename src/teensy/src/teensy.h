#ifndef TEENSY_H
#define TEENSY_H
// Wired connections between the PCB and the Teensy
// Teensy pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
// If any pinouts to the Teensy are changed, update them here:

// CONFIG
#define USE_TOM_PCB_PINOUT
// #define USE_LEO_PCB_PINOUT

#define SERVO_USE_I2C false
#define SERVO_USE_GPIO !SERVO_USE_I2C

#define GPS_USE_PI true

// CONSTANTS (usually properties of the sensor/servo itself)
#if SERVO_USE_I2C  // Are the servos connected to an adafruit PWM servo driver board?
// https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino
// I2C address for the servo driver board is 0x40 (base) + number on terminal block
#define SAIL_SERVO 0x41
#define JIB_SERVO 0x42`
#define RUDDER_SERVO 0x40
#endif

#define WATER_SENSOR1 0x36
#define WATER_SENSOR2 0x39

#define CAMERA_SERVOS_I2C 0x40

#define TEENSY_LED 20

#define SLAVE_ADDRESS 0x10

// the first byte in an i2c command, to indicate the intention of the message
// absolute movements for the camera
#define CAM_H_MOVE_ABS_CMD 0
#define CAM_V_MOVE_ABS_CMD 1
#define CAM_MOVE_ABS_CMD 2

#ifdef USE_TOM_PCB_PINOUT

// PCB -> Teensy
// Tom's AUXFETS (screw terminals) in order of edge to usb-c on pcb:
// !! 12V (definitely don't screw into the first terminal)
#define AUXFET1 38  // Connected to Teensy 5v but not 5v?
#define AUXFET2 2   // 30? 31?
#define AUXFET3 29
#define AUXFET4 28
#define AUXFET5
#define AUXFET6
// Another 12V

// PWM Pins circumventing the entire PCB and wired directly to the Teensy
#define SOLDERED_ASS_PIN1 2
#define SOLDERED_ASS_PIN2 3
#define SOLDERED_ASS_PIN3 4

// Pins repurposed from water level gpio, set of 6 pins right above teensy in order of center to
// edge GND?
#define WATER_LEVEL_3 39  // NOT PWM
#define WATER_LEVEL_2 40  // NOT PWM
#define WATER_LEVEL_1 41  // NOT PWM
// GND
// 3.3V

// Set of 2x2 pins on the edge of pcb near teensy, in order
#define TEENSY_SCL2 27
#define NETC7_2          // 5V level shift of Water level sensor?
#define TEENSY_GPIO2 23  // Not PWM?
#define NETC9_2          // 5v level shift of water level sensor 3?

// Staggered set of 2x2 pins above teensy in center
#define TEENSY_SDA2
#define NETC5_2  // ?
#define TEENSY_GPIO1 9
#define NETC8_2  // 5v level shift of water level sensor 2?

#if SERVO_USE_GPIO
#define SAIL_SERVO 23
#define JIB_SERVO -1  // 33
#define RUDDER_SERVO 24
#endif

#define WINDVANE_ENCODER_A 34
#define WINDVANE_ENCODER_B 35

#define PUMP1 AUXFET4

#define TRANSCEIVER_SERIAL &Serial1  // Serial2 wizard shit, Serial1 works through PCB?

#endif

#ifdef USE_LEO_PCB_PINOUT
#define SAIL_SERVO 23
#define JIB_SERVO 24
#define RUDDER_SERVO 9

#define TRANSCEIVER_SERIAL &Serial1

#define WINDVANE_ENCODER_A -1
#define WINDVANE_ENCODER_B -1

#define PUMP1 -1

#endif
#endif