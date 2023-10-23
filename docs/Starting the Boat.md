## Connecting the Peripherals
### Odrive
1. Connect the 3 wires of the rudder and sail odrive to the m0 and m1 screw-down headers
2. Connect the two odrive 4-pin headers to m0 and m1 (use 3v3, not 5v?)

### Compass
1. Look for [this](https://www.adafruit.com/product/1120) and connect the SCL, SDA, GND, and 3V3 pins to the [pi](https://pinout.xyz/pinout/pin5_gpio3/).
- To confirm its connected, run `i2cdetect -y 1` and look for a device on 0x1e

### Windvane
1. Thread the 6-pin header cable through the mast
2. At the top of the mast, connect the 6-pin header to the windvane
3. Connect the header to the [rotary encoder]() on the PCB, matching the direction

## Powering the Boat
1. Connect the battery pins to the PCB board (the lil "| -" plug). The odrive should power on immediately and spin
2. Flip the switch, this will power the rest of the board, including the raspberry pi

## Connecting to the Pi
#### If you know the pi's ip then connect with:
```console
ssh pi@<ip>
```
then, enter the password 'sailbot'

#### If you don't know the pi's ip:
If the Pi isn't connected a phone hotspot or our router then you'll need to connect a monitor and keyboard to the pi.


## Compiling and Running the Boat
#### Launching on Pi
```
sudo -s
cd Sailbot
. compile.sh
ros2 run sailbot [module name]
```
Available modules can be found under setup.py:entry_points, you'll usually want to run main or drivers

- Whenever the code is changed, make sure to run `. compile.sh` again
