#include "PIDController.h"

/**
 * @brief Construct a new PIDController object
 * @details Initializes the PIDController with specified gain values and setpoint.
 *          Sets the initial previous error and total error to zero.
 * @param kp The proportional gain.
 * @param ki The integral gain.
 * @param kd The derivative gain.
 * @param setpoint_value The desired setpoint for the PID controller.
 */
PIDController::PIDController(double kp, double ki, double kd, double setpoint_value) {
  // Initialize the gains and setpoint
  _Kp = kp;          // Set the proportional gain
  _Ki = ki;          // Set the integral gain
  _Kd = kd;          // Set the derivative gain
  _setpoint = setpoint_value;  // Set the desired setpoint
  _previous_error = 0.0;       // Initialize previous error to zero
  _total_error = 0.0;          // Initialize total error to zero
}

/**
 * @brief Compute the PID output
 * @details Calculate the PID output using the current angle and 
 *          the gains Kp, Ki, Kd. The output is calculated as P + I + D
 *          where P is the proportional term, I is the integral term, and
 *          D is the derivative term.
 * @param current_angle The current angle of the robot
 * @return The PID output
 */
double PIDController::compute(double current_angle) {
  double error = _setpoint - current_angle;

  // Calculate the proportional term
  double P = _Kp * error;

  // Calculate the integral term
  _total_error += error;
  double I = _Ki * _total_error;

  // Calculate the derivative term
  double derivative = error - _previous_error;
  double D = _Kd * derivative;

  // Calculate the PID output
  double output = P + I + D;

  // Update the previous error
  _previous_error = error;

  return output;
}
