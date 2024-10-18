#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
  private:
    double _Kp, _Ki, _Kd;
    double _previous_error, _total_error;
    double _setpoint;

  public:
    PIDController(double kp, double ki, double kd, double setpoint_value);
    double compute(double current_angle);
};

#endif
