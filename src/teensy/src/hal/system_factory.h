#pragma once
#include <memory>

#include "hal.h"
#include "hal_config.h"
#include "teensy.pb.h"

#ifdef HAS_SERVOS
#include "servos.h"
#endif

#ifdef HAS_RECEIVER
#include "drivers/rc_receiver/rc_receiver.h"
#if HAL_RECEIVER == RECEIVER_SBUS
#include "drivers/rc_receiver/sbus.h"
#elif HAL_RECEIVER == RECEIVER_IBUS
#include "drivers/rc_receiver/ibus.h"
#endif
#endif

#ifdef HAS_WINDVANE
#include "windvane.h"
#endif

#ifdef HAS_IMU
#include "drivers/imu/imu.h"
#if HAL_IMU == IMU_BNO055
#include "drivers/imu/bno055.h"
#elif HAL_IMU == IMU_LSM6DS
#include "drivers/imu/lsm6ds_lis3mdl.h"
#else
#error "Unknown IMU defined. Check components.h for valid options."
#endif
#endif

#ifdef HAS_GPS
#include "drivers/gps/gps.h"
#if HAL_GPS == GPS_ADAFRUIT_PA1010D
#include "drivers/gps/pa1010d.h"
#elif HAL_GPS == GPS_ADAFRUIT_PA1616S
#include "drivers/gps/pa1616s.h"
#else
#error "Unknown GPS defined. Check components.h for valid options."
#endif
#endif

#ifdef MCU_PICO2
#include <SerialPIO.h>
#endif

namespace Sailbot {
/**
 * @brief System Factory - Complete Platform Creation and Management
 *
 * Creates and manages all hardware components for the sailbot platform.
 * Uses compile-time configuration from hal_config.h and layout-specific
 * pin definitions to automatically create the appropriate hardware interfaces.
 *
 * Combines the responsibilities of system creation and platform management.
 */
class SystemFactory {
 public:
  bool initialized = false;
  // Platform components (conditionally compiled)
#ifdef HAS_SAIL
  std::unique_ptr<ServoInterface> sail_servo;
#endif
#ifdef HAS_RUDDER
  std::unique_ptr<ServoInterface> rudder_servo;
#endif
#ifdef HAS_JIB
  std::unique_ptr<ServoInterface> jib_servo;
#endif
#ifdef HAS_RECEIVER
  std::unique_ptr<RCReceiver> receiver;
#endif
#ifdef HAS_IMU
  std::unique_ptr<IMUInterface> imu;
#endif
#ifdef HAS_GPS
  std::unique_ptr<GPSInterface> gps;
#endif
#ifdef HAS_WINDVANE
  std::unique_ptr<WindVaneInterface> windvane;
#endif
#ifdef HAS_WATER_SENSORS
  std::unique_ptr<WaterSensorInterface> water_sensor1;
  std::unique_ptr<WaterSensorInterface> water_sensor2;
#endif

  /**
   * @brief Initialize all platform components
   * Uses layout-specific pin definitions and compile-time configuration
   */
  void initialize() {
    // Init Serial
    Serial.begin(115200);
#ifdef DEBUG
    while (!Serial) {
      delay(10);
    }  // Wait to allow entire Serial logs
#endif
    Serial.println("I: Initializing Sailbot Platform...");

    // Init I2C
#if defined(I2C_SDA_PIN) && defined(I2C_SCL_PIN)
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();
#endif

// Get servo specifications based on platform configuration
#ifdef HAS_SERVOS
    // Create servos based on type-safe configuration and layout pins
    // Assumes that all servos communicate over the same interface which may not always be true if
    // you have 1 servo over GPIO and another over I2C
#if HAL_SERVO_TYPE == SERVO_PROTOCOL_GPIO
#ifdef HAS_SAIL
    sail_servo = std::make_unique<GPIOServoInterface>(SAIL_SERVO_PIN, HAL_SAIL_SERVO_SPEC);
#endif
#ifdef HAS_RUDDER
    rudder_servo = std::make_unique<GPIOServoInterface>(RUDDER_SERVO_PIN, HAL_RUDDER_SERVO_SPEC);
#endif
#ifdef HAS_JIB
    jib_servo = std::make_unique<GPIOServoInterface>(JIB_SERVO_PIN, HAL_JIB_SERVO_SPEC);
#endif
#elif HAL_SERVO_TYPE == SERVO_PROTOCOL_ADAFRUIT_I2C
    // Using I2C servo driver board with per-servo specs
#ifdef HAS_SAIL
    sail_servo = std::make_unique<I2CServoInterface>(0, HAL_SAIL_SERVO_SPEC);
#endif
#ifdef HAS_RUDDER
    rudder_servo = std::make_unique<I2CServoInterface>(1, HAL_RUDDER_SERVO_SPEC);
#endif
#ifdef HAS_JIB
    jib_servo = std::make_unique<I2CServoInterface>(2, HAL_JIB_SERVO_SPEC);
#endif
#endif
#endif

// Create other components based on type-safe configuration
#ifdef HAS_IMU
#if HAL_IMU == IMU_BNO055
    imu = std::make_unique<BNO055_IMU>();
    imu->begin() ? Serial.println("I: Started BNO055 IMU")
                 : Serial.println("E: Failed to start BNO055 IMU");
#elif HAL_IMU == IMU_LSM6DS
    imu = std::make_unique<LSM6DS_IMU>();
    imu->begin() ? Serial.println("I: Started LSM6DS IMU")
                 : Serial.println("E: Failed to start LSM6DS IMU");
#endif
    imu->set_offsets(IMU_YAW_OFFSET, IMU_PITCH_OFFSET, IMU_ROLL_OFFSET);
#endif

#ifdef HAS_RECEIVER
#if HAL_RECEIVER == RECEIVER_SBUS
#if HAL_MICROCONTROLLER == MCU_TEENSY41
    // Teensy has native SBUS signal inversion so no need to flip
    receiver = std::make_unique<SBusReceiver>(TRANSCEIVER_SERIAL);
#elif HAL_MICROCONTROLLER == MCU_PICO2
#define NOPIN 255
    SerialPIO sbusSerial(NOPIN, 13);  // invert SBUS signal -> UART using PIO block
    sbusSerial.setInvertRX(true);
    receiver = std::make_unique<SBusReceiver>(TRANSCEIVER_RX_PIN);
#elif HAL_MICROCONTROLLER == MCU_CYTRON
    receiver = std::make_unique<SBusReceiver>(TRANSCEIVER_RX_PIN);
#endif
    receiver->begin() ? Serial.println("I: Started SBUS Receiver")
                      : Serial.println("E: Failed to start SBUS Receiver");
#elif HAL_RECEIVER == RECEIVER_IBUS
    receiver = std::make_unique<IBusReceiver>(TRANSCEIVER_SERIAL);
    receiver->begin() ? Serial.println("I: Started IBUS Receiver")
                      : Serial.println("E: Failed to start IBUS Receiver");
#endif
#endif

#ifdef HAS_GPS
#if HAL_GPS == GPS_ADAFRUIT_PA1616S
    gps = std::make_unique<PA1616S_GPS>(GPS_SERIAL);
    gps->begin() ? Serial.println("I: Started PA1616S GPS")
                 : Serial.println("E: Failed to start PA1616S GPS");
#elif HAL_GPS == GPS_ADAFRUIT_PA1010D
    gps = std::make_unique<PA1010D_GPS>(GPS_SERIAL);
    gps->begin() ? Serial.println("I: Started PA1010D GPS")
                 : Serial.println("E: Failed to start PA1010D GPS");
#endif
#endif

#ifdef HAS_WINDVANE
    // Use layout-defined windvane pins
#if HAL_WINDVANE == WINDVANE_ROTARY_ENCODER
    windvane = nullptr  // TODO: std::make_unique<RotaryEncoderWindVane>(WINDVANE_ENCODER_A_PIN,
                        // WINDVANE_ENCODER_B_PIN);
                        // windvane->setup();
#endif
#endif

#ifdef HAS_WATER_SENSORS
        water_sensor1 = nullptr;  // TODO: WaterSensor implementation
    water_sensor2 = nullptr;
#endif

    Serial.println("I: Platform created successfully");
    this->initialized = true;
  }

 public:
  SystemFactory() {
  }

  ~SystemFactory() = default;

  void scanI2C() {
    if (this->initialized) {
      Serial.println(
          "W: RUNNING THIS FUNCTION AFTER INITIALIZATION CAUSES I2C SENSORS TO ONLY READ 0.0");
    }

    Serial.println("D: Scanning I2C bus...");
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("D: Found device at 0x");
        Serial.println(addr, HEX);
      }
    }
  }

  /**
   * @brief Print platform configuration and initialized components
   * @return String representation of the platform
   */
  String toString() const {
    String result = "Platform: ";
    result += PLATFORM_NAME;
    result += "\n\nInitialized Components:\n";

    // Servos
#ifdef HAS_SAIL
    result += "  Sail Servo: ";
    result += (sail_servo != nullptr) ? "Initialized" : "Not initialized";
    if (sail_servo != nullptr && HAL_SERVO_TYPE == SERVO_PROTOCOL_GPIO) {
      result += " (Pin: ";
      result += SAIL_SERVO_PIN;
      result += ")";
    }
    result += "\n";
#endif

#ifdef HAS_RUDDER
    result += "  Rudder Servo: ";
    result += (rudder_servo != nullptr) ? "Initialized" : "Not initialized";
    if (rudder_servo != nullptr && HAL_SERVO_TYPE == SERVO_PROTOCOL_GPIO) {
      result += " (Pin: ";
      result += RUDDER_SERVO_PIN;
      result += ")";
    }
    result += "\n";
#endif

#ifdef HAS_JIB
    result += "  Jib Servo: ";
    result += (jib_servo != nullptr) ? "Initialized" : "Not initialized";
    if (jib_servo != nullptr && HAL_SERVO_TYPE == SERVO_PROTOCOL_GPIO) {
      result += " (Pin: ";
      result += JIB_SERVO_PIN;
      result += ")";
    }
    result += "\n";
#endif

    // IMU
    result += "  IMU: ";
#ifdef HAS_IMU
    result += (imu != nullptr) ? "Initialized" : "Not initialized";
#else
    result += "Disabled";
#endif
    result += "\n";

    // Receiver
    result += "  RC Receiver: ";
#ifdef HAS_RECEIVER
    result += (receiver != nullptr) ? "Initialized" : "Not initialized";
#else
    result += "Disabled";
#endif
    result += "\n";

    // GPS
    result += "  GPS: ";
#ifdef HAS_GPS
    result += (gps != nullptr) ? "Initialized" : "Not initialized";
#else
    result += "Disabled";
#endif
    result += "\n";

    // WindVane
    result += "  WindVane: ";
#ifdef HAS_WINDVANE
    result += (windvane != nullptr) ? "Initialized" : "Not initialized";
    if (windvane != nullptr) {
      result += " (Pins A: ";
      result += WINDVANE_ENCODER_A_PIN;
      result += ", B: ";
      result += WINDVANE_ENCODER_B_PIN;
      result += ")";
    }
#else
    result += "Disabled";
#endif
    result += "\n";

    // Water Sensors
    result += "  Water Sensors: ";
#ifdef HAS_WATER_SENSORS
    result +=
        (water_sensor1 != nullptr && water_sensor2 != nullptr) ? "Initialized" : "Not initialized";
#else
    result += "Disabled";
#endif
    result += "\n";

    return result;
  }
};
}  // namespace Sailbot