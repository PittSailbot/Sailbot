# Starting the boat
This document outlines the steps to start the boat software

## Pre-startup
Before plugging in the batteries ensure the following are plugged in:
**IMPORTANT: Do not hot plug anything on the PCB, if you want to plug something in you MUST cut power**
- 2 Motor encoder wires to the Odrive. One is labeled 0, this one should be plugged into the M0 pin headers, the black wire should go into the Gnd pin. Similarly the encoder wire labeled 1 should go into the M1 pins, the Black wire should go into Gnd (its a little harder to see the black one through the hot glue). The 3.3V pin in each set is unused.  ![Odrive Encoder Wiring](./images/odrive%20encoder%20wiring.png)

- The 2 sets of motor wires to the Odrive (3 wires in each set). These are also labeled 0 and 1, plug them into the similarly labeled wires coming out of the PCB, the other end of these wires are already screwed into the PCB. The colors of these wires do not mean anything, you can plug each wire into any 3 of the female bullet connectors. 

- The USBC cable from the mast, this plug should have a silver `T` facing up (the top side of the PCB is where the Odrive is)

- Also make sure the crows nest is plugged in, the USBC cable also has a `T` marked on it here, this time the `T` should face the 3D print, you should be able to see the black side of the USBC but not the side with the silver `T`.

- Plug in the wifi router to the USB port on the PCB directly next to the USBC port where the mast is plugged in. Do not plug it into the USB port on either of the Pis, and do not plug it into the USB port the Pi is plugged into. 

- The teensy USB cable is sometimes unplugged, make sure it is plugged into the Pi on the top of the PCB.

- Make sure the Odrive usb cable is plugged into the Rasp Pi on the top of the PCB. 

- Make sure the Power switch is in the `Off` position, try and move both switches together, they are taped together to make this easier. 

- Plug the batteries into the BMS, match the etape colors.

- If there is nothing else to plug in you can plug the BMS output into the PCB, if you later realize there is something else you forgot to plug in you must unplug the batteries.

![Wiring Diagram](./images/PCB%20wiring.png)

## Starting the Boat
1. Flip the main power switch to `On`
2. You should see the 3 Blue LEDs in the top right of the PCB, as well as the green and red LEDs on the Rasp Pi come on. If not look at [this guide](./Troublshooting.md#pcb-main-power).
3. Make sure the wifi router is on.
4. Give everything a minute to start up.
5. Connect you laptop to the `Pitt Sailbot` wifi access point. 
6. (Optional) Make sure everything is working up to this point by following the first 3 steps in [Wifi Connectivity](./Troublshooting.md#wifi-connectivity)
7. On your laptop (assumed to be windows) open powershell. You can find this by opening the start menu and searching for it
    - If you are not using windows look up how to connect using ssh on your operating system, it is a very similar process for Linux and MacOS
8. SSH into the Pi using the powershell window. The command looks like this: `ssh <user>@<Ip address>`
    - The following are subject to change:
    - Pi4: `ssh pi@192.168.8.240`
    - Pi5: `ssh sailbot@192.168.8.225`
    - The passwords for both are `sailbot`
9. Configure the Odrive (on the Pi connected to the Odrive). Make sure the motors are both able to spin a few complete rotations in each direction. With either option you should see a print out of the results with any errors generated. If there are any errors look [here](./Troublshooting.md#odrive-setup)
    - Option 1: I don't remember the name of the file but its a python file in the Utils folder that is names something like odriveSetup.py. It can be run using the following command (replace the file name with the correct one) If you type `./Utils/` (after running the cd command) and the double tap `tab` it will list the files in that folder.
    ```bash
    cd ~/Sailbot
    python3 ./Utils/<odriveSetup.py>
    ```
    - Option 2
    ```bash
    odrivetool
    # Run the following commands in the odrivetool cli 
    odrv0.clear_errors()
    odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    dump_errors(odrv0)
    # Type quit() to leave the cli
    ```
    You
10. Start the boat core using the following commands. This will only work on the pi connected to the GPIO, teensy, and Odrive. 
```bash
cd ~/Sailbot
python3 ./Utils/docker.py run
# you should now be in a docker terminal and see /workspace# to the left of your cursor, if not look for error messages
# you can type "exit" to leave the docker terminal and return to your normal terminal if you need to. 
# The following commands should be in the docker terminal
. compileDocker.sh
ros2 launch sailbot boat_all.launch.py
```
11. You should now be able to control the rudder and sail using the controller (make sure its turned on by holding the power button for a couple seconds)

## Adjusting the Sail and Rudder Zero position
1. On the right side of the controller there is a switch labled with sail/rudder offset, move this switch to the appropriate location
2. Adjust the potentiometer (the spinny knob) while the switch is in an offset position to adjust the zero point. 
3. You can move the offset switch to the None position and reset the knob if you need to do multiple rotations
