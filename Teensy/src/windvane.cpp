// Reads the rotary encoder for the windvane
#include "Adafruit_seesaw.h"

#define ENCODER_A 34
#define ENCODER_B 35
#define ENCODER_ROTATION 256  // # of values for encoder to make a full rotation

int encoderValue = 0;
int lastEncoderValue = 0;

void encoderISR() {
    // Interrupt functions to update the encoder value whenever the windvane changes angle
    // Read the current state of encoder pin B
    cli();
    // Serial.print("ISR ");
    int bState = digitalRead(ENCODER_B);
    // Serial.println(bState);
    encoderValue += (bState == HIGH) ? 1 : -1;
    encoderValue %= ENCODER_ROTATION;

    sei();
}

void setupWindvane() {
    pinMode (ENCODER_A,INPUT);
    pinMode (ENCODER_B,INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
    Serial.println("Started Windvane");
}

int readWindvane() {
  int wind_angle = map(encoderValue, 0, ENCODER_ROTATION, 0, 360)  // 0-360 degrees
}

// For testing independently
void loop () {
    delay(100);
    // did we move around?
    if (encoderValue != lastEncoderValue) {
         Serial.print("Position: ");
         Serial.println(encoderValue);
         lastEncoderValue = encoderValue;
    }
}