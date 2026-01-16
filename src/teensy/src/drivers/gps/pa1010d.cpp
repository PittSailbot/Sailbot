// Reads the Adafruit Ultimate GPS
#include "drivers/gps/pa1010d.h"

#include <Adafruit_GPS.h>
#include <Arduino.h>

#define GPSECHO false

PA1010D_GPS::PA1010D_GPS(HardwareSerial* port) : gps(port) {
}

PA1010D_GPS::PA1010D_GPS() : gps() {
}

bool PA1010D_GPS::begin() {
  // Initialize GPS over I2C at address 0x10s
  gps.begin(0x10);

  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate to 1 Hz
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  Serial.println("I: Started GPS");
  return true;
}

void PA1010D_GPS::update() {
  // Read characters from I2C - call this frequently in main loop
  char c = gps.read();
  if (GPSECHO && c) Serial.print(c);
}

bool PA1010D_GPS::read(GPSData* data) {
  // Check if a complete NMEA sentence was received
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) {
      return false;  // Failed to parse, wait for another sentence
    }
  }

  if (!gps.fix) {
    // Serial.println("I: no fix");
    return false;
  }
  data->lat = gps.latitudeDegrees;
  data->lon = gps.longitudeDegrees;
  data->speed = gps.speed * 0.5144;  // knots -> m/s
  // data->fix = gps.fix;
  // data->altitude = gps.altitude;

  return true;
}
