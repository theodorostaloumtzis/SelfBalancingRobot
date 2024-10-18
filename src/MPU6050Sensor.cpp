#include "MPU6050Sensor.h"

MPU6050Sensor::MPU6050Sensor() {}

/**
 * @brief Initialize the MPU6050 sensor
 * @details This function initializes the MPU6050 sensor's connection by calling `Wire.begin()` and `mpu.initialize()`. It sets up the I2C connection and enables the sensor. If the sensor connection fails, it prints an error message and enters an infinite loop.
 */
void MPU6050Sensor::initialize() {
  // Initialize I2C connection
  Wire.begin();

  // Initialize the MPU6050 sensor
  _mpu.initialize();

  // Check if the sensor connection is successful
  if (!_mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Enter an infinite loop to prevent code execution
  }
}

/**
 * @brief Get the pitch angle from the MPU6050 sensor
 * @details This function calculates the pitch angle of the robot using the
 *          acceleration data from the MPU6050 sensor. The pitch angle is
 *          calculated using the arctangent of the ratio of the x-axis to the
 *          z-axis acceleration values. The result is returned as a double
 *          value representing the angle in degrees.
 *
 * @return The pitch angle in degrees
 */
double MPU6050Sensor::getPitchAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate pitch angle in degrees
  double pitch_angle = atan2(ax, az) * 180 / PI;  
  return pitch_angle;
}

