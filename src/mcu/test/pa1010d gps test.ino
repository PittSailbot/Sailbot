// Test code for Adafruit GPS That Support Using I2C
//
// This code shows how to parse data from the I2C GPS
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true  // Changed to true for debugging

uint32_t timer = millis();

#define I2C_SDA 0
#define I2C_SCL 1

void setup() {
  // while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Adafruit I2C GPS library basic test!");

  // Initialize I2C bus
  Wire.setSDA(I2C_SDA);  // Virtual GP (not actual PIN number)
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  delay(1000);  // Give GPS time to boot and I2C to stabilize

  // Scan I2C bus to find devices
  Serial.println("Scanning I2C bus (Wire)...");
  int deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  if (deviceCount == 0) {
    Serial.println("  No I2C devices found on Wire bus!");
  }
  Serial.println();

  // Test direct I2C communication
  Serial.print("Testing I2C connection to GPS at 0x10... ");
  Wire.beginTransmission(0x10);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("OK!");

    // Try to read some bytes directly
    Serial.print("Requesting data from GPS... ");
    int available = Wire.requestFrom(0x10, 32);
    Serial.print(available);
    Serial.println(" bytes available");

    if (available > 0) {
      Serial.print("Data: ");
      while (Wire.available()) {
        char c = Wire.read();
        Serial.print(c);
      }
      Serial.println();
    }
  } else {
    Serial.print("FAILED with error ");
    Serial.println(error);
  }
  delay(500);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(0x10);  // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loop()  // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // Debug: Check if we're getting any data
  static unsigned long lastCharTime = 0;
  static int charCount = 0;
  if (c) {
    charCount++;
    lastCharTime = millis();
    if (GPSECHO) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data

    // Store the sentence first before it gets consumed
    char *nmea = GPS.lastNMEA();
    Serial.print("NMEA: ");
    Serial.println(nmea);

    if (!GPS.parse(nmea))  // parse the stored sentence
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis();  // reset the timer

    // Debug: Show character count
    Serial.print("\nChars received: ");
    Serial.print(charCount);
    Serial.print(" | ");

    Serial.print("Time: ");
    if (GPS.hour < 10) {
      Serial.print('0');
    }
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    if (GPS.minute < 10) {
      Serial.print('0');
    }
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    if (GPS.seconds < 10) {
      Serial.print('0');
    }
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");
    Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4);
      Serial.println(GPS.lon);
      Serial.print("Speed (knots): ");
      Serial.println(GPS.speed);
      Serial.print("Angle: ");
      Serial.println(GPS.angle);
      Serial.print("Altitude: ");
      Serial.println(GPS.altitude);
      Serial.print("Satellites: ");
      Serial.println((int)GPS.satellites);
    }
  }
}