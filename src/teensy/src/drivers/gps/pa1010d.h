#pragma once

#include <Adafruit_GPS.h>

#include "drivers/gps/gps.h"

class PA1010D_GPS : public GPSInterface {
 private:
  Adafruit_GPS gps;

 public:
  PA1010D_GPS(HardwareSerial* port);
  bool begin() override;
  bool read(GPSData* data) override;
};