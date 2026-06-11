// Reads the Adafruit Ultimate GPS
#include "drivers/gps/pa1010d.h"

#include <Adafruit_GPS.h>
#include <Arduino.h>

#include "elapsedMillis.h"

elapsedMillis last_warn_gps = 100000; // time hack to warn at boot

#define GPSECHO false

PA1010D_GPS::PA1010D_GPS(HardwareSerial* port) : gps(port) {
}

PA1010D_GPS::PA1010D_GPS() : gps(&Wire) {
}

bool PA1010D_GPS::begin() {
  // Initialize GPS over I2C at address 0x10
  if (!this->gps.begin(0x10)) {
    Serial.println("E: Failed to find PA1010D GPS... Check your wiring or I2C ADDR!");
    return false;
  }

  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate to 1 Hz
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  this->initialized = true;
  return true;
}

void PA1010D_GPS::update() {
  // Read characters from I2C - call this frequently in main loop
  char c = gps.read();
  if (GPSECHO && c) Serial.print(c);

  if (gps.newNMEAreceived()) {
    gps.parse(gps.lastNMEA());
  }
}

bool PA1010D_GPS::read(GPSData* data) {
  if (!this->initialized && last_warn_gps > 60000) {
    last_warn_gps = 0;
    Serial.println("W: Trying to read from uninitialized GPS. Trying to reinit.");
    this->begin();
  }

  if (!gps.fix) {
    if (last_warn_gps > 60000) {
      last_warn_gps = 0;
      Serial.println("W: No GPS fix");
    }
    return false;
  }
  data->lat = gps.latitudeDegrees;
  data->lon = gps.longitudeDegrees;
  data->speed = gps.speed * 0.5144;  // knots -> m/s
  // data->fix = gps.fix;
  // data->altitude = gps.altitude;

  return true;
}
