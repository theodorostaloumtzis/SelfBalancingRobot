#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
  private:
    double Kp, Ki, Kd;
    double previous_error, total_error;
    double setpoint;

  public:
    PIDController(double kp, double ki, double kd, double setpoint_value);
    double compute(double current_angle);
};

#endif
