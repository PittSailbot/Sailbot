// Reads the acceleration, gyroscope and magnometer from the IMU

#include "imu.h"

// Which of our IMUs to use
#define USE_BNO055_IMU
// #define USE_LSM6DS_IMU

#if defined(USE_BNO055_IMU)
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
class BNO055_IMU : public IMUInterface {
 private:
  Adafruit_BNO055 bno;

 public:
  bool setup() override {
    bno = Adafruit_BNO055(55);
    if (!bno.begin()) {
      Serial.println("E: Failed to find BNO055 IMU... Check your wiring or I2C ADDR!");
      return false;
    }

    bno.setExtCrystalUse(true);

    this->initialized = true;
    return true;
  }

  void update() override {
    // BNO055 has its own fusion, no need for separate filter
    return;
  }

  bool read(IMU* imu) override {
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
};

#elif defined(USE_LSM6DS_IMU)

#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
#include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
// #include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
// #include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

class LSM6DS_IMU : public IMUInterface {
 private:
  // pick your filter! slower == better quality output
  Adafruit_NXPSensorFusion filter;  // slowest
  // Adafruit_Madgwick filter;  // faster than NXP
  //  Adafruit_Mahony filter;  // fastest/smalleset
  uint32_t timestamp;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif
 public:
  bool setup() override {
    if (!cal.begin()) {
      Serial.println("E: Failed to initialized calibration helper");
      return false;
    } else if (!cal.loadCalibration()) {
      Serial.println("E: No calibration loaded/found");
      return false;
    }

    if (!init_sensors()) {
      Serial.println("E: Failed to find LSM6DS IMU... Check your wiring or I2C ADDR!");
      return false;
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();

    setup_sensors();

    filter.begin(FILTER_UPDATE_RATE_HZ);
    this->timestamp = millis();

    this->initialized = true;
    Serial.println("I: Started IMU");
    return true;
  }

  void update() override {
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
    filter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
  }

  bool read(IMU* imu) override {
    if (!this->initialized) {
      return false;
    }

    if ((millis() - this->timestamp) > (1000 / FILTER_UPDATE_RATE_HZ)) {
      this->update();
    }

    imu->roll = filter.getRoll();
    imu->pitch = filter.getPitch();
    imu->yaw = filter.getYaw();

    return true;
  }
};

#endif

IMUInterface* imu_sensor = nullptr;

int setupIMU() {
#if defined(USE_BNO055_IMU)
  imu_sensor = new BNO055_IMU();
#elif defined(USE_LSM6DS_IMU)
  imu_sensor = new LSM6DS_IMU();
#else
  Serial.println("E: No IMU type defined in header");
  return 1;
#endif

  if (!imu_sensor->setup()) {
    Serial.println("E: Failed to start IMU");
    return 1;
  }

  return 0;
}

bool readIMU(IMU* imu_data) {
  if (!imu_sensor->initialized) {
    return false;
  }

  imu_sensor->read(imu_data);
  return true;
}