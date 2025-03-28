// Wired connections between the PCB and the Teensy
// Teensy pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
// If any pinouts to the Teensy are changed, update them here:

// PCB -> Teensy
#define AUXFET1 38
#define AUXFET2 2
#define AUXFET3 29
#define AUXFET4 28

// Teensy -> Sensors/Controls
#define PUMP1 AUXFET4
#define PUMP2 AUXFET3
#define PUMP3 AUXFET2

#define SAIL_SERVO 23 // TODO: Set to servo control pin
#define JIB_SERVO -1 // TODO: Set to servo control pin
#define RUDDER_SERVO -1 // TODO: Set to servo control pin

#define WATER_SENSOR1 41
#define WATER_SENSOR2 40
#define WATER_SENSOR3 39

#define WATER_SENSOR1_INSTALLED false
#define WATER_SENSOR2_INSTALLED true
#define WATER_SENSOR3_INSTALLED true

#define WINDVANE_ENCODER_A 34
#define WINDVANE_ENCODER_B 35

#define SLAVE_ADDRESS 0x10

// the first byte in an i2c command, to indicate the intention of the message
// absolute movements for the camera
#define CAM_H_MOVE_ABS_CMD 0
#define CAM_V_MOVE_ABS_CMD 1
#define CAM_MOVE_ABS_CMD 2
