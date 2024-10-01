// Reads the rotary encoder for the windvane
#include "Adafruit_seesaw.h"
#include "teensy.h"
#include "windvane.h"

#define ENCODER_ROTATION 256  // # of values for encoder to make a full rotation

int encoderValue = 0;
int lastEncoderValue = 0;
/*
displayedEncoderValue = 0;
minEncoderValue = 0;
maxEncoderValue = 0;
*/
bool hasChanged = false;

void encoderISR() {
    // Interrupt functions to update the encoder value whenever the windvane changes angle
    // Read the current state of enc oder pin B
    cli();
    //lastEncoderValue = encoderValue;
    int bState = digitalRead(WINDVANE_ENCODER_B);
    encoderValue += (bState == HIGH) ? 1 : -1;
    /*
    
    
    */
    encoderValue %= ENCODER_ROTATION;
    hasChanged = true;

    sei();
}

extern void setupWindVane() {
    pinMode (WINDVANE_ENCODER_A,INPUT);
    pinMode (WINDVANE_ENCODER_B,INPUT);

    attachInterrupt(digitalPinToInterrupt(WINDVANE_ENCODER_A), encoderISR, RISING);
    Serial.println("Started WindVane");
}

extern bool readWindVane(WindVane* windvane) {
    windvane->wind_angle = map(encoderValue, 0, ENCODER_ROTATION, 0, 360);  // 0-360 degrees
    return hasChanged;
}

// For testing independently
// void loop () {
//     delay(100);
//     // did we move around?
//     if (true || encoderValue != lastEncoderValue) {
//          Serial.print("Position: ");
//          Serial.println(encoderValue);
//          lastEncoderValue = encoderValue;
//     }
// }