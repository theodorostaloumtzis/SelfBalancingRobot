#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include "PIDController.h"
#include "MPU6050Sensor.h"
#include "MotorDriver.h"

// WiFi and WebServer variables
const char* ssid = "COSMOTE-875731";  // Your WiFi SSID
const char* password = "hda4252txcac5nrm";  // Your WiFi Password
WebServer server(80);  // Create a web server on port 80

// Initialize objects
MPU6050Sensor _mpu_sensor;
PIDController _pid(30.0, 0.0, 0.5, 0.0);  // Kp, Ki, Kd, setpoint (horizontal)
MotorDriver _motor_driver(16, 17, 18, 19, 5, 4);  // Adjusted pins

double _current_pitch = 0.0;
double _pid_output = 0.0;

// Desired IP address
IPAddress _requestedIP(192, 168, 60, 125);  // Set the requested IP address
IPAddress _gateway(192, 168, 1, 1); // Set your gateway IP (usually the router's IP)
IPAddress _subnet(255, 255, 255, 0);  // Set the subnet mask

unsigned long _previous_millis = 0;
const long _interval = 2; // interval in milliseconds

/**
 * @brief Controls the motors based on the received command
 * @details This function controls the motors based on the received command.
 *          It sets the motor speeds according to the command and returns a
 *          confirmation response.
 * @param command The command to control the motors.
 * @return A confirmation response indicating the action taken.
 */
String controlMotors(String command) {
  int motorSpeed = 0;
  String response = "Command executed: " + command;

  // Handle the different commands
  if (command == "FORWARD") {
    motorSpeed = 200;  // Set forward speed
    _motor_driver.setMotorSpeed(motorSpeed, motorSpeed);
    response += " - Moving Forward";
  } else if (command == "BACKWARD") {
    motorSpeed = -200; // Set backward speed
    _motor_driver.setMotorSpeed(motorSpeed, motorSpeed);
    response += " - Moving Backward";
  } else if (command == "LEFT") {
    _motor_driver.setMotorSpeed(-150, 150); // Adjust speeds for turning left
    response += " - Turning Left";
  } else if (command == "RIGHT") {
    _motor_driver.setMotorSpeed(150, -150); // Adjust speeds for turning right
    response += " - Turning Right";
  } else if (command == "STOP") {  // Handle STOP command
    _motor_driver.setMotorSpeed(0, 0); // Stop the motors
    response += " - Stopping";
  } else {
    response += " - Invalid command";
  }

  return response;  // Return confirmation message
}

/**
 * @brief Handle motor control commands
 * @details This function handles incoming motor control commands. It checks
 *          if the command argument is present in the request, and if it is, it
 *          calls the controlMotors function to control the motors and obtain
 *          a confirmation response. It then sends the confirmation response
 *          back to the client.
 */
void handleCommand() {
  if (server.hasArg("command")) {
    String command = server.arg("command");

    // Print the received command to the serial monitor
    Serial.print("Command received: ");
    Serial.println(command);

    // Control the motors and obtain a confirmation response
    String response = controlMotors(command);
    server.send(200, "text/plain", response);  // Send confirmation back to the client
  } else {
    // Handle invalid requests
    server.send(400, "text/plain", "No command received");
  }
}

/**
 * @brief Handle status request
 * @details This function includes the current pitch angle and PID output in the response and sends it to the server.
 */
void handleStatus() {
  // Include current pitch and PID output in the response
  String response = "Pitch Angle: " + String(_current_pitch, 2);
  response += "\nPID Output: " + String(_pid_output, 2);

  server.send(200, "text/plain", response);  // Send pitch and PID values
}

/**
 * @brief Initialize the robot's WiFi connection and server
 * @details This function initializes the robot's WiFi connection using the
 *          specified IP address, gateway, and subnet. It then sets up
 *          endpoints for motor control commands and pitch/PID status requests,
 *          and starts the server.
 */
void setup() {
  Serial.begin(9600);
  _mpu_sensor.initialize();
  _motor_driver.initialize();
  
  // Connect to WiFi
  WiFi.config(_requestedIP, _gateway, _subnet);  // Set the requested IP address, gateway, and subnet
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Define endpoints
  server.on("/command", handleCommand);  // Handle motor control commands
  server.on("/status", handleStatus);    // Handle pitch and PID status requests
  server.begin();  // Start the server

  Serial.println("Server started");
  _motor_driver.setMotorSpeed(0, 0);
}

/**
 * @brief Main loop function
 * @details Handles incoming client requests and updates the PID output 
 *          at a specified interval. The PID output is calculated using the
 *          current pitch angle of the robot. The loop also prints the pitch
 *          angle and PID output to the serial monitor.
 */
void loop() {
  server.handleClient();  // Handle incoming client requests

  unsigned long current_millis = millis();
  if (current_millis - _previous_millis >= _interval){
    // Get the pitch angle and compute the PID output
    _current_pitch = _mpu_sensor.getPitchAngle();  // Get the pitch angle
    _pid_output = _pid.compute(_current_pitch);  // Use pitch angle in PID

    // Print values to the serial monitor
    Serial.print("Pitch Angle: ");
    Serial.print(_current_pitch);
    Serial.print("\tPID Output: ");
    Serial.println(_pid_output);
  } 
}
