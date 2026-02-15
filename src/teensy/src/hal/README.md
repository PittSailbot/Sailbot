# Sailbot HAL System
This is Sailbot's custom HAL (Hardware Abstraction Layer) for making code hardware-independent.

## Directory Structure
```text
/
├── hal/                      # Configs, source code, etc. for building the hardware layer
│   ├── components.h          # New hardware sensors, servos, etc. definitions
│   ├── hal_config.h          # Which platform to use, its capabilities, and components used definitions
│   ├── hal.h                 # Interface for the HAL
│   ├── system_factory.h      # Backend to build the HAL interface using the components defined in hal_config.h
├── layouts/                  # Various microcontroller pinouts for our PCBs
```

## How to Extend the HAL
### Adding a new PCB
1. In `/layouts` add a new header file and fill in the pinouts using other files as an example
   - Pinout definitions *must* be spelled the same
2. Include the new layout in `system_factory.h`
3. Add the PCB definition to `hal_config.h` and add an ifdef check to include the relevant layout header

### Adding a new component
1. Add/extend a new interface in the relevant component header (`gps.h`, `imu.h`, etc.)
```cpp
// GeneralComponentInterface is a 'parent' of the specific interfaces. It is used in the HAL to generalize interacting with *any* of those sensors. You can also use it to reuse code for common functions (ex. servo.center()).
// SpecificComponentInterface handles differences between component types. (ex. Adafruit sensors might have different ways to initialize the sensor and different code needed to read from it)
class SpecificComponentInterface : public GeneralComponentInterface { // Specific inherits from General interface
private:
// Define anything that is driver-specific and other people shouldn't need to worry about or use
int i2c_addr;

public:
SpecificComponentInterface(int arg1); // Constructor
SpecificComponentInterface(int arg1, int optional_arg); // Overloaded constructor
// Define methods required by the General interface
void begin() override; // Override replaces the General interface's method with what you will define in the .cpp
}
```
2. Write the driver code in the relevant .cpp
3. Add the component to the relevant enum in `components.h` (ex. `IMU::BNO055`)
   - If the component is something we've never had before, then make a new enum and in `hal_config` add a `HAS_X` for it
4. In `system_factory.h:initialize()` add another case for that enum with code on how to make your specific component
5. Update any platforms in `hal_config` with the new component

### Adding a new platform (PCB + Components)
1. In `hal_config.h` add a new name for the platform
```cpp
// Uncomment ONE platform:
// #define TOM_PCB_DAN_MARINA      // Main boat + Teensy 4.1 + Tom PCB
// #define LEO_PCB_DAN_MARINA      // Mini boat + Pico + Leo PCB
// #define DRAGONFORCE_65            // Mini boat autonomy testing
// #define PLATFORM_TEST_BREADBOARD // Test setup + Teensy + Breadboard
#define YOUR_PLATFORM_NAME
```
2. Add the components used following the others as a template
```cpp
#ifdef YOUR_PLATFORM_NAME
  // Define sensor, components, etc. here
  #define RECEIVER_TYPE ReceiverType::SBUS
  #define PCB_SPEC PCBSpec::LEO_PCB

  #define SAIL_SERVO_SPEC ServoSpecs::BILDA
  #define RUDDER_SERVO_SPEC ServoSpecs::BILDA

  #define PLATFORM_NAME "Dan Marina (Leo PCB)"
#endif
```

### Using the HAL
In `main.cpp`, all you should need to worry about is building the HAL and interacting with its interface `hal.h`.

To build the HAL:
```cpp
#include "hal/system_factory.h"

auto platform = Sailbot::SystemFactory::createPlatform();
```

To use components:
```cpp
// Control servos (pins automatically mapped from layout)
#if HAS_SAIL_SERVO
  platform->sail_servo->write(45);
#endif

// Read sensors (only compiled if available)
#if HAS_GPS
    bool gps_ok = platform->readGPS(&gps_data);
#endif

#if HAS_WINDVANE
    bool wind_ok = platform->readWindVane(&wind_data);
#endif
```

# Design Decisions
Why did we make a HAL? What problems does it try to solve? What are its limitations?

## The Problem
So what's the point of all this stuff? As our club has grown we've had to support an increasing amount of legacy and new electronics. So far we've had (as of 2026):

- 4 different PCBs (soon to be 6+) along with handling undefined breadboard testing
  - Leo PCB
  - Tom PCB
  - Stan PCB 2
  - Stan PCB 1
- 3+ different microcontroller boards
  - Pi Pico 2
  - Cytron Motion 2350 Pro (for testing)
  - Teensy 4.1
- 3 Different control methods
  - Servos--some with 2 or 3 for jib control and either over GPIO or I2C using Adafruit Servo Driver
  - DC Motors
  - Stepper Servos
- 2 GPSs
  - Adafruit PA1010D
  - Adafruit PA1616S
- 2 IMUs
  - Adafruit BNO085
  - Adafruit LSM6DSOX
- 2 RC Receivers with two different controller schemes
  - FrSky X9 Lite
  - FlySky IA6B

You don't need to do the permutations to tell that there's a lot of different ways to configure our boats. Plus with how robotics goes, there's a very real chance that something fries and we're forced to use an older sensor or even different PCB. Swapping components shouldn't require a full code rewrite.

## The Goal
Ultimately this library is supposed to make your life easier. To do so, I've taken the following architectural choices:

#### 1. Separate application logic from drivers
Higher-level code shouldn't care about the differences between our RC controllers. All that matters is that it can read the controller's channel data. If you want to add a new receiver just make a driver using the RCReceiver interface and have it output the channel data. To higher-level code in main, it doesn't matter if the receiver uses SBUS, UART, etc. or what function you need to call to read from it, it all works the same.

#### 2. Standardize PCB/MCU definitions
Our PCBs all have different mappings for which pin connects to which component or header. Some PCBs also have more features than others, like having water level sensors or more servos. Defining each of the PCB/layouts makes the code compatible with different configurations.

#### 3. Conditional compilation
There's no need to compile or run code to check our water level sensors if they're not
even plugged into the boat. This has the added benefit of eliminating unexpected bugs
from disabled modules.


## Why Custom?
*Surely someone's made a library for all this before right?*

Yes, and no. Our HAL is built off Arduino which handles very low-level abstractions like GPIO, UART, Serial and other interfaces. However, interfaces for sensors like IMU, GPS, water level, etc. don't exist officially and some of the interfaces that do exist like Servos are dumb (angles > 200 are treated as PWM which led to an obscure bug with our 1800 degree winch servo).

Some popular RTOS's (Real-time Operating System) like Zephyr handles a ton of abstraction, but doing so requires a full code rewrite, has a steep learning curve for new members, and makes its painful to break the abstraction rules compared to the freedom of bare-metal. With ours, you can completely ignore this library if you want (but don't).

Since most of our electronics already have built in drivers from their vendors (Adafruit, FrSky, etc.) which themselves abstract from Arduino, we can pretty much ignore low-level stuff like how a PWM signal is actually generated and how that varies between different microcontrollers. This simplifies *a lot* and makes our work significantly easier.


### Limitations
- There's no differentiation between the boat and PCB used. This means that if the PCB supports GPS/IMU/WindVane it'll use all of those even if you're just testing RC and those components aren't plugged in. This can cause errors unless you temporarily disable those components by commenting them out.
- The current implementation is locked into using protobuf by expecting each abstracted component to return some data type defined in the .proto. This isn't really best practice, but it does reduce the chance of breaking changes. If you want to abstract the communication protocol to allow JSON/Serial/etc. between the MCU and the Pi you would need to use structs or convert proto to your desired format (inefficient).
- There's currently no way to have both GPIO & I2C servos at the same time (ie. rudder connected over PWM, sail over I2C)
- RCData .proto assumes a single controller layout even though our two controllers have different switches
- Switch UP/DOWN macro, etc. assumed to be same across receivers
- Stuck using the preprocessor #ifdef for specific driver inclusion
  - enums don't seem to work with intellisense 'dead-code' checking (ServoType::GPIO doesn't invalidate ServoType::ADAFRUIT_I2C_DRIVER)