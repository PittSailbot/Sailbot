# Powering the Boat
1. Connect the battery pins to the PCB board, the odrive should power on immediately and spin
2. Flip the switch, this will power the rest of the board, including the raspberry pi

# Connecting to the Pi
### If you know the pi's ip then connect with:
```console
ssh pi@<ip>
```
then, enter the password 'sailbot'

### If you don't know the pi's ip:
If the Pi isn't connected a phone hotspot or our router then you'll need to connect a monitor and keyboard to the pi.


# Copying Code between Pi and PC

# Compiling and Running the Boat
### Launching on Pi
```
sudo -s
. init.sh
ros2 run sailbot [module name]
```
Available modules are main and drivers
### Compiling code changes
```
sudo -s
. compile.sh
```
and then
```
ros2 run sailbot [module name]
```
or
```
ros2 launch sailbot [launch file]
```
```
