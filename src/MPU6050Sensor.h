#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <Wire.h>
#include <MPU6050.h>

class MPU6050Sensor {
  private:
    MPU6050 _mpu;

  public:
    MPU6050Sensor();
    void initialize();
    double getPitchAngle();  // Function to get pitch angle
};

#endif
