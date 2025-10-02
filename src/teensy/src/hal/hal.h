#pragma once

// Forward declarations to reduce compile time
class ServoInterface;
class IMUInterface;
class RCReceiver;
class GPSInterface;
class WindVaneInterface;
class WaterSensorInterface;

namespace Sailbot {
/**
 * @brief Hardware Abstraction Layer - Platform-specific hardware creation
 *
 * The HAL provides a unified interface for creating hardware drivers
 * while abstracting platform-specific implementation details.
 */
class HAL {
 public:
  virtual ~HAL() = default;

  // Core hardware creation methods
  virtual std::unique_ptr<ServoInterface> createServo(uint8_t pin_or_channel) = 0;
  virtual std::unique_ptr<IMUInterface> createIMU() = 0;
  virtual std::unique_ptr<RCReceiver> createReceiver() = 0;
  virtual std::unique_ptr<GPSInterface> createGPS() = 0;
  virtual std::unique_ptr<WindVaneInterface> createWindVane() = 0;
  virtual std::unique_ptr<WaterSensorInterface> createWaterSensor(uint8_t i2c_address) = 0;

  // Platform information
  virtual const char* getPlatformName() = 0;
  virtual bool hasI2CServoDriver() = 0;
  virtual uint8_t getMaxServos() = 0;
};
}  // namespace Sailbot