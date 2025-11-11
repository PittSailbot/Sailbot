// Driver for Adafruit BNO055 9-DOF Absolute Orientation IMU
#include "drivers/imu/bno055.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
BNO055_IMU::BNO055_IMU() : bno(55, BNO055_I2C_ADDR, &Wire) {
}

bool BNO055_IMU::begin() {
  if (!this->bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("E: Failed to find BNO055 IMU... Check your wiring or I2C ADDR!");
    return false;
  }

  this->initialized = true;
  return true;
}

bool BNO055_IMU::read(IMU* imu) {
  if (!this->initialized) {
    Serial.println("W: Trying to read from uninitialized IMU");
    return false;
  }

  // Check sensor calibration state
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  if (gyro != 3) {
    // If this happens, leave the IMU stationary for a few seconds
    Serial.println("W: IMU Gyro not fully calibrated, readings will be inaccurate!");
  }
  if (mag != 3) {
    // If this happens, try moving the imu around in a figure-8 pattern
    Serial.println("W: IMU Magnetometer not fully calibrated, readings will be inaccurate!");
  }

  if (mag < 2 || gyro < 2) {
    Serial.println("W: IMU not fully calibrated, too unreliable to rely on readings");
    // IMU yaw is unreliable at 0 or 1 calibration state
    // Could still read from the sensor, but its usually better to just wait for better accuracy
    return false;
  }

  // Read sensor data
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData,
      accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // Additional unused IMU sensor data
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // DEBUG DATA
  // printEvent(&orientationData);
  // printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  // printEvent(&gravityData);

  // Apply offsets to account for IMU mounting orientation
  imu->yaw = fmod(orientationData.orientation.x + this->yaw_offset + 360.0, 360.0);
  imu->pitch = fmod(orientationData.orientation.y + this->pitch_offset + 360.0, 360.0);
  imu->roll = fmod(orientationData.orientation.z + this->roll_offset + 360.0, 360.0);

  return true;
}
