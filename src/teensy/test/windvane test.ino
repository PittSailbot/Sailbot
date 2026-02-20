#include <SPI.h> // SPI library
#include <Wire.h>



const int MISO_PIN = 4; // GP 4 / Pin 6 (SPI0 RX)
const int csPin    = 5; // GP 5 / Pin 7 (SPI0 CSn)
const int SCK_PIN  = 2; // GP 2 / Pin 3(SPI0 SCK)
// Connect MOSI to VCC

SPISettings sensorSettings(1000000, MSBFIRST, SPI_MODE1);


void setup() {
  Serial.begin(9600); 
  
  // Set the CS pin as an output and set it HIGH (inactive)
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  
  // use SPI library to assign RX on pico as MISO (Master-In, Slave_Out). We don't need to assign the MOSI (Master-Out, Slave-In) pin because
  // the pico doesnt need to send anything back to the sensor
  SPI.setMISO(MISO_PIN);
  // use SPI library to set the clock pin
  SPI.setSCK(SCK_PIN);


  // Initialize the SPI library *after* setting the pins
  SPI.begin();
}

void loop() {
  //begin the transaction with the previous settings, and set the slave select pin (csPin) to LOW to indicate that
  //we will be using this device to receive data
  SPI.beginTransaction(sensorSettings);
  digitalWrite(csPin, LOW);

  // Read the two bytes
  byte byte1 = SPI.transfer(0x00);
  byte byte2 = SPI.transfer(0x00);
  
  //set the slave select pin back to HIGH after we have received our data and end the transaction
  digitalWrite(csPin, HIGH);
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
    Serial.println("Sensor Error!");
  } else if (parityError) {
    // The data was corrupted in transit
    Serial.println("Parity Error!");
  } else {
  
    // Use a mask to extract the data and convert it to an angle
    // Mask 0b0011111111111111
    int angleData = rawData & 0x3FFF;

    float angleInDegrees = (float)angleData / 16384.0f * 360.0f;
    
    // Print the valid angle data
    Serial.println(angleInDegrees);
  }
  
  delay(100); // Wait a moment before reading again
}