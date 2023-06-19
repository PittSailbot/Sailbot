# SailBot
Main codebase for the University of Pittsburgh's SailBot Club (2019-2024). *[Competition info here](https://www.sailbot.org/)*.

Onenote notebook containing boat startup steps, documentation and passwords can be found [here](https://pitt-my.sharepoint.com/:o:/r/personal/tad85_pitt_edu/Documents/Sailbot%20Stuff?d=w0d1afb3f4ab44df2b02186d8015ae380&csf=1&web=1&e=uvn9TV)
- Some info is out of date

## Installation
1. Install [Python 3.10](https://www.python.org/downloads/release/python-31011/)
2. Install [Git](https://git-scm.com/downloads)
3. Install your code editor of choice:
- We recommend [Pycharm](https://www.jetbrains.com/pycharm/download/#section=windows) or [VSCode](https://code.visualstudio.com/download)
  - Pycharm has lots of helpful features and is tailored specifically for Python
  - VSCode is light and works for any language but requires lots of plugins to be installed to compare to Pycharm
  - Personally, I use Pycharm for Python and VSCode for any other programming language

4. In the terminal, navigate to the directory where you want to install the repository and enter:
```
git clone https://github.com/PittSailbot/Sailbot.git
```
5. In File Explorer, right-click the project directory and click 'Open with Pycharm/VSCode'
6. In the app's integrated terminal, install the required dependencies with:
```
pip install -r dev-requirements.txt
```

## Scripts
Located in src/sailbot/sailbot
### Logic
- **boatMain** - main loop controlling every aspect of the boat

#### Events
Direct boat behavior depending on which [competition challenge](https://www.sailbot.org/wp-content/uploads/2022/05/SailBot-2022-Events.pdf) the boat is participating in
- **search** - Autonomously find a buoy in a 100m search radius
- **stationKeeping** - Keep the boat within a 40x40m box for a specific amount of time
- **endurance** - Sail for as far and long as possible around a box autonomously or RC
- **precisionNavigation** - Autonomously round buoys in a triangle and exit within a 3m gap between two buoys

### Sensors/Controls
Scripts which interface with the mechanical parts of the boat. Virtual versions of each sensor exist for debugging with fake values.
- **GPS** - boat position
- **compass** - boat heading
- **windvane** - wind direction
- **camera** - RGB optical camera
- **cameraServos** - pitch and yaw servos controlling camera movement
- **boatMovement** - rudder/sail controls and controlling the boat's movement
- **Odrive** - motors used in rudder and sail control
- **transceiver** - wireless communication to shore

### Utils
Miscellaneous functions used by the boat
- **constants** - config containing all static parameters used by the boat
- **utils** - commonly used functions for anything
- **boatMath** - common functions for working with coordinates and angles
- **eventUtils** - common functions used in events
- **objectDetection** - AI buoy detection from an image

### Debug
Scripts used to test boat behavior
- **boatSim** - simulates how the boat moves in a virtual environment

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
