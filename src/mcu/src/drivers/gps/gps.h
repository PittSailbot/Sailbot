#pragma once
#include "mcu.pb.h"

/**
 * @brief Common interface for all GPS implementations
 *
 * This abstract base class defines the minimal interface that all GPS drivers
 * must implement. It provides a consistent API for initialization and data reading
 * regardless of the underlying hardware.
 */
class GPSInterface {
 public:
  bool initialized = false;

  virtual ~GPSInterface() = default;

  /**
   * @brief Initialize the GPS hardware
   * 普通话：初始化 GPS 硬件和底层通信接口。
   * @return true if initialization succeeded, false otherwise
   */
  virtual bool begin() = 0;

  /**
   * @brief Update GPS - call frequently in main loop to read incoming data
   * 普通话：在主循环中高频调用，用来持续读取 GPS/NMEA 数据流。
   * This method should be called as often as possible to ensure the GPS
   * buffer is being read and NMEA sentences are being assembled.
   */
  virtual void update() = 0;

  /**
   * @brief Read current GPS data
   * 普通话：把当前解析好的 GPS 数据写入 protobuf 结构。
   * @param gps Pointer to GPS protobuf to populate with data
   * @return true if data was successfully read, false otherwise
   */
  virtual bool read(GPSData* gps) = 0;
};
