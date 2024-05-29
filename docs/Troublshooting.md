# Troubleshooting
This guide helps solve problems related to the boat

## PCB Main Power
The batteries are plugged in but nothing is lighting up
1. Check the voltage of the plug you are connecting to PCB. It should be >10.8V. If it is not then the batteries are dead or nearly dead and the BMS might be preventing them from being used to prevent damage, you will need to charge the batteries or use different ones. 

2. Some of the deans plugs do not fit correctly together, For example the batteries will not make proper contact with the PCB when plugged in directly. Try using jumper wires to connect the battery to the PCB. **BE VERY CAREFUL!** Switching the Ground and V+ wire could destroy part or all of the PCB. Accidentally touching the battery alligator clips while connecting them to the PCB Could destroy the battery. 

## Wifi Connectivity
1. Make sure the wifi router is running and has a green light for the wifi signal
2. Make sure your device is connected to the `Pitt Sailbot` wifi network
3. Open a browser and type `192.168.8.1` into the address bar. The password is `sailboot`. On the left there should be a tab labeled clients, after clicking on it you should see the clients currently connected. If you've gotten to this point you laptop will definitly be one of the listed devices, you should also see the Pi(s) and their Ip addresses. 
4. If you do not see the Pi(s) connected unplug the Pi power cable from the PCB (do not turn off the entire PCB, this is the only time you should do this). **Do not unplug the GPIO cable** (The GPIO cable is the wide black cable)
5. Wait a moment and plug the Pi back in, make sure it has a green light blinking (or steady). If only the red light shows up the is a problem with the Pi. If not light comes up it might not be getting power
6. If you see the green light on the Pi then give it a little while and try again to connect. 
    - Sometimes it takes a minute or two for the Pi to show up (or disappear from) clients at `192.168.8.1`


## Odrive Setup
- Unbalanced Phases: This seems to happen with axis1 sometimes, just swap the 3 motor wires around so they connect to a different female connector. You may have to try several different options before it works, run the calibration sequence with each try. It is ok to unplug and replug these wires if and only if the motors calibration sequence failed for the axis you are unplugging AND you have not reset the axis using odrv0.clear_errors() 
- Encoder CPR mismatch: This happens somewhat randomly, make sure the wires are plugged all the way in and the encoders are firmly connected to the motor. Spin the motor a little and make sure there is not too much resistance (if there is spray some wd40 or something) Even still this happens on occasion, just run the calibration again. 
- These are the only two common errors, if anything else comes up while running the calibration try asking chatGPT for help. 