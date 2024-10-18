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

/**
 * @brief Set the proportional gain
 * @details Set the proportional gain of the PID controller. The
 *          proportional gain is used to calculate the proportional
 *          term of the PID output. The proportional term is
 *          calculated as KP * error where error is the difference
 *          between the setpoint and the current angle.
 * @param kp The proportional gain
 */
void PIDController::setKp(double kp) {
  _Kp = kp;
}

/**
 * @brief Set the integral gain
 * @details Set the integral gain of the PID controller. The
 *          integral gain is used to calculate the integral term
 *          of the PID output. The integral term is calculated as
 *          KI * total_error where total_error is the sum of all
 *          errors since the last time the integral term was
 *          calculated.
 * @param ki The integral gain
 */
void PIDController::setKi(double ki) {
  _Ki = ki;
}

/**
 * @brief Set the derivative gain
 * @details Set the derivative gain of the PID controller. The
 *          derivative gain is used to calculate the derivative term
 *          of the PID output. The derivative term is calculated as
 *          KD * (current_error - previous_error)
 * @param kd The derivative gain
 */
void PIDController::setKd(double kd) {
  _Kd = kd;
}

/**
 * @brief Set the setpoint of the PID controller
 * @details Set the setpoint of the PID controller. The setpoint is the
 *          desired angle of the robot. The PID controller will
 *          calculate the PID output based on the difference between
 *          the setpoint and the current angle.
 * @param setpoint_value The desired angle of the robot.
 */
void PIDController::setSetpoint(double setpoint_value) {
  _setpoint = setpoint_value;
}
