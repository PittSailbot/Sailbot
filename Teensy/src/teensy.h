// Wired connections between the PCB and the Teensy
// If any pinouts to the Teensy are changed, update them here:

// PCB
#define AUXFET1 38
#define AUXFET2 2
#define AUXFET3 29
#define AUXFET4 28

// Teensy Pinouts
#define PUMP1 AUXFET4
#define PUMP2 AUXFET3
#define PUMP3 AUXFET2
#define WATER_SENSOR1 41
#define WATER_SENSOR2 40
#define WATER_SENSOR3 39

#define WINDVANE_ENCODER_A 34
#define WINDVANE_ENCODER_B 35