#include "MotorDriver.h"
#include <Arduino.h>

/**
 * @brief Construct a MotorDriver object
 * @details
 * A constructor for the MotorDriver class which sets the pin numbers for the two motors and their enable pins.
 *
 * @param m1_pin1 The pin number for motor 1's direction 1.
 * @param m1_pin2 The pin number for motor 1's direction 2.
 * @param m2_pin1 The pin number for motor 2's direction 1.
 * @param m2_pin2 The pin number for motor 2's direction 2.
 * @param en_pin1 The pin number for motor 1's enable pin.
 * @param en_pin2 The pin number for motor 2's enable pin.
 */
MotorDriver::MotorDriver(int m1_pin1, int m1_pin2, int m2_pin1, int m2_pin2, int en_pin1, int en_pin2)
{
  // Set the pin numbers for the two motors and their enable pins
  _motor1_pin1 = m1_pin1; // pin number for motor 1's direction 1
  _motor1_pin2 = m1_pin2; // pin number for motor 1's direction 2
  _motor2_pin1 = m2_pin1; // pin number for motor 2's direction 1
  _motor2_pin2 = m2_pin2; // pin number for motor 2's direction 2
  _enable_pin1 = en_pin1; // pin number for motor 1's enable pin
  _enable_pin2 = en_pin2; // pin number for motor 2's enable pin
}

/**
 * @brief Initializes the motor driver's pins
 * @details Sets the pin modes for all of the motor driver's pins to OUTPUT.
 *          This function is called in the constructor of the MotorDriver class.
 *          It ensures that the pins are properly configured for motor control.
 */
void MotorDriver::initialize()
{
  // Set the pin modes for the two motors and their enable pins
  // to OUTPUT. This is necessary because the motor pins must be
  // configured as outputs before they can be used to control the
  // motors.
  pinMode(_motor1_pin1, OUTPUT);
  pinMode(_motor1_pin2, OUTPUT);
  pinMode(_motor2_pin1, OUTPUT);
  pinMode(_motor2_pin2, OUTPUT);
  pinMode(_enable_pin1, OUTPUT);
  pinMode(_enable_pin2, OUTPUT);
}

/**
 * @brief Sets the speed of the two motors
 * @details Sets the direction of rotation of the two motors and the speed of the two motors.
 *
 * @param motor1Speed The speed of motor 1, positive for forward and negative for backward.
 * @param motor2Speed The speed of motor 2, positive for forward and negative for backward.
 */
void MotorDriver::setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed > 0) {
    // Set the direction of motor 1 to forward
    digitalWrite(_motor1_pin1, HIGH);
    digitalWrite(_motor1_pin2, LOW);
  } else {
    // Set the direction of motor 1 to backward
    digitalWrite(_motor1_pin1, LOW);
    digitalWrite(_motor1_pin2, HIGH);
  }
  // Set the speed of motor 1 using PWM
  analogWrite(_enable_pin1, abs(motor1Speed));

  if (motor2Speed > 0) {
    // Set the direction of motor 2 to forward
    digitalWrite(_motor2_pin1, HIGH);
    digitalWrite(_motor2_pin2, LOW);
  } else {
    // Set the direction of motor 2 to backward
    digitalWrite(_motor2_pin1, LOW);
    digitalWrite(_motor2_pin2, HIGH);
  }
  // Set the speed of motor 2 using PWM
  analogWrite(_enable_pin2, abs(motor2Speed));
}
