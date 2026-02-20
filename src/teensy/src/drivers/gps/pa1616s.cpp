// Reads the Adafruit Ultimate GPS
#include "drivers/gps/pa1616s.h"

#include <Adafruit_GPS.h>
#include <Arduino.h>

PA1616S_GPS::PA1616S_GPS(HardwareSerial* port) : gps(port) {
}

bool PA1616S_GPS::begin() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if (!gps.begin(9600)) {
    return false;
  }

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("I: Started GPS");
  return true;
}

bool PA1616S_GPS::read(GPSData* data) {
  char c = gps.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (gps.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(gps.lastNMEA());    // this also sets the newNMEAreceived() flag to false
    if (!gps.parse(gps.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
      return false;  // we can fail to parse a sentence in which case we should just wait for
                     // another
  }

  // if (!gps.fix){
  //   Serial.println("no fix");
  //   return false;
  // }
  data->lat = gps.latitudeDegrees;
  data->lon = gps.longitudeDegrees;
  data->speed = gps.speed * 0.5144;  // knots -> m/s
  // data->fix = gps.fix;
  // data->altitude = gps.altitude;

  return true;
}