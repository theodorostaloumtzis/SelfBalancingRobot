#include "MPU6050Sensor.h"

MPU6050Sensor::MPU6050Sensor() {}

void MPU6050Sensor::initialize() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

// Calculate the pitch angle (Y-axis)
double MPU6050Sensor::getPitchAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate pitch angle in degrees
  double pitch_angle = atan2(ax, az) * 180 / PI;  
  return pitch_angle;
}
