#pragma once

#include <Adafruit_GPS.h>

#include "drivers/gps/gps.h"

class PA1010D_GPS : public GPSInterface {
 private:
  Adafruit_GPS gps;

 public:
  PA1010D_GPS(HardwareSerial* port);  // Configure GPS over UART
  PA1010D_GPS();                      // Configure GPS over I2C
  bool begin() override;
  void update() override;  // Call frequently to read GPS data
  bool read(GPSData* data) override;
};