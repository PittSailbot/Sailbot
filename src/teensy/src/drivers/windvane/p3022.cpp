// Reads the CALT P3022 Absolute Potentiometer
#include "drivers/windvane/p3022.h"

#include <Arduino.h>

static SPISettings sensorSettings(1000000, MSBFIRST, SPI_MODE1);

P3022_WindVane::P3022_WindVane(int miso_pin, int cs_pin, int sck_pin)
    : miso_pin(miso_pin), cs_pin(cs_pin), sck_pin(sck_pin) {
}

bool P3022_WindVane::begin() {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  SPI.setMISO(miso_pin);
  SPI.setSCK(sck_pin);

  SPI.begin();

  this->initialized = true;
  return true;
}

bool P3022_WindVane::read(WindVane* windvane) {
  if (!this->initialized) {
    Serial.println("W: Trying to read from uninitialized WindVane");
    return false;
  }

  // begin the transaction with the previous settings, and set the slave select pin (cs_pin) to LOW
  // to indicate that we will be using this device to receive data
  SPI.beginTransaction(sensorSettings);
  digitalWrite(this->cs_pin, LOW);

  // Read the two bytes
  byte byte1 = SPI.transfer(0x00);
  byte byte2 = SPI.transfer(0x00);

  // set the slave select pin back to HIGH after we have received our data and end the transaction
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();

  // Combine the two bytes into one 16-bit packet
  int rawData = (byte1 << 8) | byte2;

  // --- Check 1: The 'EF' bit (Bit 14) ---
  // Mask is 0b0100000000000000
  bool errorFlag = (rawData & 0x4000);

  // --- Check 2: The 'PAR' bit (Even Parity) ---
  // We count all the '1's in the entire 16-bit packet.
  // __builtin_popcount() is a fast, built-in function that
  // "counts the population" of '1's in an integer.
  int oneCount = __builtin_popcount(rawData);

  // For EVEN parity, an ODD count is an error.
  // (oneCount % 2) != 0 will be 'true' if the count is odd.
  bool parityError = (oneCount % 2) != 0;

  // --- Final Validation ---
  if (errorFlag) {
    // The sensor reported an error
    Serial.println("E: WindVane Sensor Error!");
    return false;
  } else if (parityError) {
    // The data was corrupted in transit
    Serial.println("W: WindVane Parity Error!");
    return false;
  } else {
    // Use a mask to extract the data and convert it to an angle
    // Mask 0b0011111111111111
    int angleData = rawData & 0x3FFF;

    float angleInDegrees = (float)angleData / 16384.0f * 360.0f;

    windvane->wind_angle = angleInDegrees;
    return true;
  }
}