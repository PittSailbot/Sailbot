# SailBot
Main codebase for the University of Pittsburgh's SailBot Club (2019-24)

*[Competition info here](https://www.sailbot.org/)*

## Guide
Onenote notebook containing boat startup steps, documentation and passwords can be found [here](https://pitt-my.sharepoint.com/:o:/r/personal/tad85_pitt_edu/Documents/Sailbot%20Stuff?d=w0d1afb3f4ab44df2b02186d8015ae380&csf=1&web=1&e=uvn9TV)

## Installation
In the terminal, navigate to the directory where you want to install the repository and enter
```
git clone https://github.com/SailBotPitt/SailBot.git
```

Install required dependencies by entering
```
pip install -r requirements.txt
```
*this requires python 3.10 to be installed*

## ROS2
The code uses ROS2 as our framework.
### Launching on Pi
```
sudo -s
. init.sh
ros2 run sailbot [module name]
```

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

## Docker
Docker allows everyone to run the code in the same environment limiting computer dependent conflicts. It also ensures that all necessary libraries and external files are accounted for on the github. (necessary for the CI/CD pipeline)

### Running Docker
First you must install docker on your computer  
(https://docs.docker.com/desktop/install/windows-install/)  
The docker image can be created using the following commands in a terminal which is located in the same folder as this readme file. This may take awhile the first time (docker desktop must be running)
```
py ./docker.py create
```
launch the container using
```
py ./docker.py run
```
connect to an already running container using
```
py docker.py connect
```

### Running code in Docker container
the process is very similar to the pi, the only exception is that sudo -s is not necessary (nor will it work)
```
. compile.sh
```
or, if its already been compiled in another terminal:
```
. init.sh
```
and then
```
ros2 run sailbot [module name]
```
or
```
ros2 launch sailbot [launch file]
```

## Scripts
Located in src/sailbot/sailbot
### Logic
- **boatMain** - main loop controlling every aspect of the boat

#### Events
Direct boat behavior depending on which [competition challenge](https://www.sailbot.org/wp-content/uploads/2022/05/SailBot-2022-Events.pdf) the boat is participating in
- **manualControl** # TODO
- **search** - 
- **stationKeeping** - 
- **endurance** - 
- **precisionNavigation** - 
- **collisionAvoidance** - 

### Sensors/Controls
Scripts which interface with the mechanical parts of the boat and provide abstracted functions used by 
- **GPS** - boat position
- **compass** - boat heading
- **windvane** - wind direction
- **camera** - RGB optical camera
- **cameraServos** - pitch and yaw servos controlling camera movement
- **drivers** - controls motors for rudder and sail
- **transceiver** - wireless communication to shore

### Utils
Miscellaneous functions used by the boat
- **constants** - config containing all static parameters used by the boat
- **boatMath** - common functions for converting between coordinates and angles
- **objectDetection** - AI buoy detection from an image
- **Odrive** - used to calibrate motor speed and limits
- **eventUtils** - common functions used in events

### Debug
Scripts used to test boat behavior
- **boatSim** - simulates how the boat moves in a virtual environment
