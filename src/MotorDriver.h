#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

class MotorDriver {
  private:
    int motor1_pin1, motor1_pin2, motor2_pin1, motor2_pin2, enable_pin1, enable_pin2;

  public:
    MotorDriver(int m1_pin1, int m1_pin2, int m2_pin1, int m2_pin2, int en_pin1, int en_pin2);
    void initialize();
    void setMotorSpeed(int motor1Speed, int motor2Speed);
};

#endif
