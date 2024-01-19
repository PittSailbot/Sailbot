#include "sbus.h"

// Replace with data.NUM_CH for dynamic
#define USED_CHANNELS 9
#define RC_LOW 172
#define RC_HIGH 1811

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2);
/* SBUS data */
bfs::SbusData data;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();
  Serial.println("Started...");
}

void loop () {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    printControllerState(data);
    /* Display lost frames and failsafe data */
    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
    /* Set the SBUS TX data to the received data */
    sbus_tx.data(data);
    /* Write the data to the servos */
    sbus_tx.Write();
  }
}

void printControllerState(bfs::SbusData data) {
  /* EXPECTED RC CONTROLLER FORMAT
  Max and min trim thresholds are within +-10. They do not effect the max/min value. They only offset the "center" value.
  - Down/Left reads ~172-180 (Converted and serialized 0)
    - Front-facing switches are reversed and read 0 on up
  - Center reads ~980-1000 (Converted and serialized at 50)
  - Up/Right reads ~1795-1811 (Converted and serialized at 100)
  RC receiver still repeats the last controller state when it is off/connection lost. Could cause issues.

  Channels:
  0 - left_analog_y
  1 - right_analog_x
  2 - right_analog_y
  3 - left_analog_x
  4 - front_left_switch1
  5 - front_left_switch2
  6 - front_right_switch
  7 - top_left_switch
  8 - top_right_switch
  9 - potentiometer
  10-16 - UNUSED
  */

  /*for (int8_t i = 0; i < USED_CHANNELS; i++) {
    Serial.print(map(data.ch[i], 172, 1811, 0, 100));
    Serial.print("\t");
  }*/
  Serial.print(map(data.ch[0], RC_LOW, RC_HIGH, 0, 100));
  Serial.print("\t");
  Serial.print(map(data.ch[1], RC_LOW, RC_HIGH, 0, 100));
  Serial.print("\t");
  Serial.print(map(data.ch[2], RC_LOW, RC_HIGH, 0, 100));
  Serial.print("\t");
  Serial.print(map(data.ch[3], RC_LOW, RC_HIGH, 0, 100));
  Serial.print("\t");
  Serial.print(map(data.ch[4], RC_LOW, RC_HIGH, 0, 2));
  Serial.print("\t");
  Serial.print(map(data.ch[5], RC_LOW, RC_HIGH, 0, 2));
  Serial.print("\t");
  Serial.print(map(data.ch[6], RC_LOW, RC_HIGH, 0, 2));
  Serial.print("\t");
  Serial.print(map(data.ch[7], RC_LOW, RC_HIGH, 0, 1));
  Serial.print("\t");
  Serial.print(map(data.ch[8], RC_LOW, RC_HIGH, 0, 1));
  Serial.print("\t");
  Serial.print(map(data.ch[9], RC_LOW, RC_HIGH, 0, 100));
  Serial.print("\t");
}