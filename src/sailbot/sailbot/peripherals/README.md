# Peripherals
This folder contains all the sensors/controls that are connected to the Raspberry Pi. This is not ALL
of our sensors. The rest of our sensors are connected to the Teensy and their code is located in src/teensy.

- **transceiver.py** - Reads serialized data from the Teensy, which includes the RC controller and misc sensors
- **camera.py** - RGB pan-tilt camera
- **Odrive.py/motorDrivers.py** - Controls for the sail and rudder motors