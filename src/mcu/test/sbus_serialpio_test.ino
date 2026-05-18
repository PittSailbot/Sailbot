/**
 * SBUS SerialPIO Test for Pico2/RP2350
 *
 * Tests reading SBUS receiver using SerialPIO with signal inversion.
 * The RP2350 hardware UART doesn't support inversion, but SerialPIO does.
 *
 * Wiring:
 * - SBUS receiver signal -> GP9 (RX)
 * - SBUS receiver power -> 5V
 * - SBUS receiver ground -> GND
 *
 * SBUS Protocol:
 * - 100000 baud, 8 data bits, even parity, 2 stop bits
 * - Inverted signal (hence we use setInvertRX)
 * - 25 byte packets
 */

#include <Arduino.h>
#include <SerialPIO.h>

// SerialPIO with RX on GP9, no TX needed
// Use NOPIN (255) for TX since we only need RX
SerialPIO sbusSerial(NOPIN, 9);

// SBUS packet structure
#define SBUS_PACKET_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

uint8_t sbusBuffer[SBUS_PACKET_SIZE];
uint8_t bufferIndex = 0;

// Channel data (16 channels)
uint16_t channels[16];

bool parseSbusPacket() {
  // Verify header and footer
  if (sbusBuffer[0] != SBUS_HEADER) {
    return false;
  }

  // Parse 16 channels (11 bits each, packed into bytes 1-22)
  channels[0] = ((sbusBuffer[1] | sbusBuffer[2] << 8) & 0x07FF);
  channels[1] = ((sbusBuffer[2] >> 3 | sbusBuffer[3] << 5) & 0x07FF);
  channels[2] = ((sbusBuffer[3] >> 6 | sbusBuffer[4] << 2 | sbusBuffer[5] << 10) & 0x07FF);
  channels[3] = ((sbusBuffer[5] >> 1 | sbusBuffer[6] << 7) & 0x07FF);
  channels[4] = ((sbusBuffer[6] >> 4 | sbusBuffer[7] << 4) & 0x07FF);
  channels[5] = ((sbusBuffer[7] >> 7 | sbusBuffer[8] << 1 | sbusBuffer[9] << 9) & 0x07FF);
  channels[6] = ((sbusBuffer[9] >> 2 | sbusBuffer[10] << 6) & 0x07FF);
  channels[7] = ((sbusBuffer[10] >> 5 | sbusBuffer[11] << 3) & 0x07FF);
  channels[8] = ((sbusBuffer[12] | sbusBuffer[13] << 8) & 0x07FF);
  channels[9] = ((sbusBuffer[13] >> 3 | sbusBuffer[14] << 5) & 0x07FF);
  channels[10] = ((sbusBuffer[14] >> 6 | sbusBuffer[15] << 2 | sbusBuffer[16] << 10) & 0x07FF);
  channels[11] = ((sbusBuffer[16] >> 1 | sbusBuffer[17] << 7) & 0x07FF);
  channels[12] = ((sbusBuffer[17] >> 4 | sbusBuffer[18] << 4) & 0x07FF);
  channels[13] = ((sbusBuffer[18] >> 7 | sbusBuffer[19] << 1 | sbusBuffer[20] << 9) & 0x07FF);
  channels[14] = ((sbusBuffer[20] >> 2 | sbusBuffer[21] << 6) & 0x07FF);
  channels[15] = ((sbusBuffer[21] >> 5 | sbusBuffer[22] << 3) & 0x07FF);

  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("SBUS SerialPIO Test for Pico2");
  Serial.println("==============================");
  Serial.println("Using GP9 as RX with signal inversion");
  Serial.println();

  // Configure SerialPIO with inversion
  sbusSerial.setInvertRX(true);

  // SBUS uses 100000 baud, 8E2 (8 data bits, even parity, 2 stop bits)
  sbusSerial.begin(100000, SERIAL_8E2);

  Serial.println("SerialPIO initialized. Waiting for SBUS data...");
  Serial.println("Move your controller sticks to see values change.");
  Serial.println();
}

unsigned long lastPrintTime = 0;
unsigned long lastByteTime = 0;
bool receivingPacket = false;

void loop() {
  // Read available bytes
  while (sbusSerial.available()) {
    uint8_t byte = sbusSerial.read();
    unsigned long now = millis();

    // SBUS packets have a gap between them (~4ms)
    // If we haven't received a byte in >3ms, start a new packet
    if (now - lastByteTime > 3) {
      bufferIndex = 0;
    }
    lastByteTime = now;

    // Look for header byte to start packet
    if (bufferIndex == 0 && byte != SBUS_HEADER) {
      continue;  // Skip until we find header
    }

    // Store byte in buffer
    if (bufferIndex < SBUS_PACKET_SIZE) {
      sbusBuffer[bufferIndex++] = byte;
    }

    // Check if we have a complete packet
    if (bufferIndex == SBUS_PACKET_SIZE) {
      if (parseSbusPacket()) {
        // Print channel values every 100ms
        if (now - lastPrintTime > 100) {
          Serial.print("CH1-4: ");
          for (int i = 0; i < 4; i++) {
            Serial.print(channels[i]);
            Serial.print(" ");
          }
          Serial.print(" | CH5-8: ");
          for (int i = 4; i < 8; i++) {
            Serial.print(channels[i]);
            Serial.print(" ");
          }
          Serial.println();
          lastPrintTime = now;
        }
      }
      bufferIndex = 0;
    }
  }

  // Warn if no data received
  static unsigned long lastWarnTime = 0;
  if (millis() - lastByteTime > 5000 && millis() - lastWarnTime > 5000) {
    Serial.println("Warning: No SBUS data received in 5 seconds");
    Serial.println("Check wiring: SBUS signal -> GP9");
    lastWarnTime = millis();
  }
}
