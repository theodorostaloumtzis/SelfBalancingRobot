#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int m1_pin1, int m1_pin2, int m2_pin1, int m2_pin2, int en_pin1, int en_pin2) {
  motor1_pin1 = m1_pin1;
  motor1_pin2 = m1_pin2;
  motor2_pin1 = m2_pin1;
  motor2_pin2 = m2_pin2;
  enable_pin1 = en_pin1;
  enable_pin2 = en_pin2;
}

void MotorDriver::initialize() {
  pinMode(motor1_pin1, OUTPUT);
  pinMode(motor1_pin2, OUTPUT);
  pinMode(motor2_pin1, OUTPUT);
  pinMode(motor2_pin2, OUTPUT);
  pinMode(enable_pin1, OUTPUT);
  pinMode(enable_pin2, OUTPUT);
}

void MotorDriver::setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed > 0) {
    digitalWrite(motor1_pin1, HIGH);
    digitalWrite(motor1_pin2, LOW);
  } else {
    digitalWrite(motor1_pin1, LOW);
    digitalWrite(motor1_pin2, HIGH);
  }
  analogWrite(enable_pin1, abs(motor1Speed));

  if (motor2Speed > 0) {
    digitalWrite(motor2_pin1, HIGH);
    digitalWrite(motor2_pin2, LOW);
  } else {
    digitalWrite(motor2_pin1, LOW);
    digitalWrite(motor2_pin2, HIGH);
  }
  analogWrite(enable_pin2, abs(motor2Speed));
}
