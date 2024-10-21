#include <Wire.h>
#include <MPU6050.h>
#include "PIDController.h"
#include "MPU6050Sensor.h"
#include "MotorDriver.h"

// Initialize objects
MPU6050Sensor _mpu_sensor;
PIDController _pid(30.0, 0.0, 0.5, 0.0);  // Kp, Ki, Kd, setpoint (horizontal)
MotorDriver _motor_driver(16, 17, 18, 19, 5, 4);  // Adjusted pins

double _current_pitch = 0.0;
double _pid_output = 0.0;

unsigned long _previous_millis = 0;
const long _interval = 250; // interval in milliseconds

void setup() {
  Serial.begin(9600);

  // Initialize the MPU6050 sensor
  _mpu_sensor.initialize();
  Serial.println("Initializing MPU6050 sensor...");

  // Initialize the motor driver
  _motor_driver.initialize();
  Serial.println("Initializing motor driver...");

  // Set initial motor speed to zero
  _motor_driver.setMotorSpeed(0, 0);
  Serial.println("Setting initial motor speed to zero");

  // Wait for 5 seconds 
  delay(5000);
}

void loop() {
  // Get the current time in milliseconds
  unsigned long current_millis = millis();

  // Check if the interval has passed since the last measurement
  if (current_millis - _previous_millis >= _interval) {
    // Get the current pitch angle from the MPU6050 sensor
    _current_pitch = _mpu_sensor.getPitchAngle();

    // Calculate the PID output
    _pid_output = _pid.compute(_current_pitch);

    // Control the motors based on PID output
    int motorSpeed = static_cast<int>(_pid_output); // Convert PID output to motor speed
    _motor_driver.setMotorSpeed(motorSpeed, motorSpeed); // Adjust motor speeds based on PID output

    // Print the pitch angle and PID output to the serial monitor
    Serial.print("Pitch Angle: ");
    Serial.print(_current_pitch);
    Serial.print("\tPID Output: ");
    Serial.println(_pid_output);

    // Update previous millis
    _previous_millis = current_millis;
  }
}
