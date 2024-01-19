Contains all the code and libraries for our teensy microcontroller. Changing these scripts *will not* automatically update them. You must connect to the specific board over USB and upload the new script using Arduino IDE.

# Teensy
- Reads the RC controller's state and serializes to transceiver.py
- (WIP) Reads the water level sensors and serializes *only* when water is detected in any of the sensors