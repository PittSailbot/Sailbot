// Reads the acceleration, gyroscope and magnometer from the IMU
#include "imu.h"

#include <Adafruit_AHRS.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Sensor_Calibration.h>
#include <utility/imumaths.h>

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
BNO055_IMU::BNO055_IMU() : bno(55) {
}

bool BNO055_IMU::begin() {
  if (!bno.begin()) {
    Serial.println("E: Failed to find BNO055 IMU... Check your wiring or I2C ADDR!");
    return false;
  }

  bno.setExtCrystalUse(true);

  this->initialized = true;
  return true;
}

bool BNO055_IMU::read(IMU* imu) {
  if (!this->initialized) {
    return false;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating powsersint data */
  // Serial.print("X: ");
  // Serial.print(event.orientation.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);
  // Serial.println("");

  imu->yaw = event.orientation.x;
  imu->pitch = event.orientation.y;
  imu->roll = event.orientation.z;
  // Acceleration can be collected but unused
  // imu->accel_x = event.acceleration.x

  return true;
}

// ===== LSM6DS_LIS3MDL Driver =====
LSM6DS_IMU::LSM6DS_IMU() : lsm6ds() {
}

bool LSM6DS_IMU::init_sensors() {
  if (!this->lsm6ds.begin_I2C() || !this->lis3mdl.begin_I2C()) {
    return false;
  }

  this->accelerometer = this->lsm6ds.getAccelerometerSensor();
  this->gyroscope = this->lsm6ds.getGyroSensor();
  this->magnetometer = &this->lis3mdl;

  // Set sensor ranges and data rates
  this->lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  this->lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  this->lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  this->lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  this->lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  this->lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  this->lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  this->lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  return true;
}

bool LSM6DS_IMU::begin() {
  if (!this->cal.begin()) {
    Serial.println("E: Failed to initialized calibration helper");
    return false;
  } else if (!this->cal.loadCalibration()) {
    Serial.println("E: No calibration loaded/found");
    return false;
  }

  if (!init_sensors()) {
    Serial.println("E: Failed to find LSM6DS IMU... Check your wiring or I2C ADDR!");
    return false;
  }

  this->filter.begin(FILTER_UPDATE_RATE_HZ);
  this->timestamp = millis();

  this->initialized = true;
  return true;
}

void LSM6DS_IMU::update() {
  if (!this->initialized) {
    return;
  }

  float gx, gy, gz;

  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  this->filter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
}

bool LSM6DS_IMU::read(IMU* imu) {
  if (!this->initialized) {
    return false;
  }

  if ((millis() - this->timestamp) > (1000 / FILTER_UPDATE_RATE_HZ)) {
    this->update();
  }

  imu->roll = this->filter.getRoll();
  imu->pitch = this->filter.getPitch();
  imu->yaw = this->filter.getYaw();

  return true;
}