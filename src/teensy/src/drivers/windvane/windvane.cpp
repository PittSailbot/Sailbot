// // Reads the rotary encoder for the windvane
// #include "drivers/windvane/windvane.h"

// #include "Adafruit_seesaw.h"
// #include "teensy.h"

// #define ENCODER_ROTATION 256

// volatile int encoderValue = 0;
// int lastEncoderValue = 0;
// volatile bool hasChanged = false;

// // Filter variables
// float filteredEncoderValue = 0;
// const float FILTER_ALPHA = 0.15;  // Adjust between 0.0 (heavy filtering) and 1.0 (no filtering)

// void encoderISR() {
//   // Interrupt functions to update the encoder value whenever the windvane changes angle
//   // Read the current state of encoder pin B
//   cli();

//   int bState = digitalRead(WINDVANE_ENCODER_B);
//   encoderValue += (bState == HIGH) ? 1 : -1;

//   // Handle wrap-around properly
//   if (encoderValue >= ENCODER_ROTATION) {
//     encoderValue -= ENCODER_ROTATION;
//   } else if (encoderValue < 0) {
//     encoderValue += ENCODER_ROTATION;
//   }

//   hasChanged = true;

//   sei();
// }

// int filterWindVane(int rawValue)
// {
//   int diff = rawValue - (int)filteredEncoderValue;

//   if (diff > ENCODER_ROTATION / 2) {
//     diff -= ENCODER_ROTATION;
//   } else if (diff < -ENCODER_ROTATION / 2) {
//     diff += ENCODER_ROTATION;
//   }

//   filteredEncoderValue += FILTER_ALPHA * diff;

//   // Normalize back to 0-255 range
//   if (filteredEncoderValue >= ENCODER_ROTATION) {
//     filteredEncoderValue -= ENCODER_ROTATION;
//   } else if (filteredEncoderValue < 0) {
//     filteredEncoderValue += ENCODER_ROTATION;
//   }

//   return (int)filteredEncoderValue;
// }

// extern void setupWindVane() {
//   pinMode(WINDVANE_ENCODER_A, INPUT);
//   pinMode(WINDVANE_ENCODER_B, INPUT);

//   // Initialize filter with current position
//   filteredEncoderValue = encoderValue;

//   attachInterrupt(digitalPinToInterrupt(WINDVANE_ENCODER_A), encoderISR, RISING);
//   Serial.println("I: Started WindVane");
// }

// extern bool readWindVane(WindVane* windvane) {
//   if (hasChanged) {
//     int filteredValue = filterWindVane(encoderValue);

//     windvane->wind_angle = map(filteredValue, 0, ENCODER_ROTATION, 0, 360);

//     hasChanged = false;
//     return true;
//   }
//   return false;
// }

// // For testing independently
// // void loop () {
// //     delay(100);
// //     WindVane wv;
// //     if (readWindVane(&wv)) {
// //          Serial.print("Raw: ");
// //          Serial.print(encoderValue);
// //          Serial.print(" Filtered Angle: ");
// //          Serial.println(wv.wind_angle);
// //     }
// // }