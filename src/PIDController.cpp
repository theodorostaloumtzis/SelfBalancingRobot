#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double setpoint_value) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  setpoint = setpoint_value;
  previous_error = 0.0;
  total_error = 0.0;
}

double PIDController::compute(double current_angle) {
  double error = setpoint - current_angle;
  double P = Kp * error;
  total_error += error;
  double I = Ki * total_error;
  double derivative = error - previous_error;
  double D = Kd * derivative;
  double output = P + I + D;
  previous_error = error;
  return output;
}
