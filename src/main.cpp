#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include "PIDController.h"
#include "MPU6050Sensor.h"
#include "MotorDriver.h"

// WiFi and WebServer variables
const char* ssid = "COSMOTE-875731";
const char* password = "hda4252txcac5nrm";
WebServer server(80);  // Create a web server on port 80

// Initialize objects
MPU6050Sensor mpu_sensor;
PIDController pid(30.0, 0.0, 0.5, 0.0);  // Kp, Ki, Kd, setpoint (horizontal)
MotorDriver motor_driver(16, 17, 18, 19, 5, 4);  // Adjusted pins

double current_pitch = 0.0;
double pid_output = 0.0;

// Function to control motors based on received command
String controlMotors(String command) {
  int motorSpeed = 0;
  String response = "Command executed: " + command;

  if (command == "FORWARD") {
    motorSpeed = 200;  // Set forward speed
    motor_driver.setMotorSpeed(motorSpeed, motorSpeed);
    response += " - Moving Forward";
  } else if (command == "BACKWARD") {
    motorSpeed = -200; // Set backward speed
    motor_driver.setMotorSpeed(motorSpeed, motorSpeed);
    response += " - Moving Backward";
  } else if (command == "LEFT") {
    motor_driver.setMotorSpeed(-150, 150); // Adjust speeds for turning left
    response += " - Turning Left";
  } else if (command == "RIGHT") {
    motor_driver.setMotorSpeed(150, -150); // Adjust speeds for turning right
    response += " - Turning Right";
  } else {
    motor_driver.setMotorSpeed(0, 0); // Stop motors if no valid command
    response += " - Stopping";
  }

  return response;  // Return confirmation message
}

// Handle motor control commands
void handleCommand() {
  if (server.hasArg("command")) {
    String command = server.arg("command");

    // Print the received command to the serial monitor
    Serial.print("Command received: ");
    Serial.println(command);

    String response = controlMotors(command);  // Control motors and get confirmation response
    server.send(200, "text/plain", response);  // Send confirmation back to the client
  } else {
    server.send(400, "text/plain", "No command received");
  }
}

// Handle status request (pitch angle and PID output)
void handleStatus() {
  // Include current pitch and PID output in the response
  String response = "Pitch Angle: " + String(current_pitch, 2);
  response += "\nPID Output: " + String(pid_output, 2);

  server.send(200, "text/plain", response);  // Send pitch and PID values
}

void setup() {
  Serial.begin(9600);
  mpu_sensor.initialize();
  motor_driver.initialize();
  
  // Connect to WiFi
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
}

void loop() {
  server.handleClient();  // Handle incoming client requests

  // Get the pitch angle and compute the PID output
  current_pitch = mpu_sensor.getPitchAngle();  // Get the pitch angle
  pid_output = pid.compute(current_pitch);  // Use pitch angle in PID

  // Apply the PID output to the motors for self-balancing
  int motorSpeed = constrain(pid_output, -255, 255);
  motor_driver.setMotorSpeed(motorSpeed, motorSpeed);

  // Print values to the serial monitor
  Serial.print("Pitch Angle: ");
  Serial.print(current_pitch);
  Serial.print("\tPID Output: ");
  Serial.println(pid_output);
  
  delay(10);  
}
