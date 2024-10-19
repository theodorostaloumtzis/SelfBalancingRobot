#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESPmDNS.h> // Include mDNS library
#include "PIDController.h"
#include "MPU6050Sensor.h"
#include "MotorDriver.h"

// WiFi and WebServer variables
const char* ssid = "Your_SSID";  // Your WiFi SSID
const char* password = "Your_WiFi_Password";  // Your WiFi Password
WebServer server(80);  // Create a web server on port 80

// Initialize objects
MPU6050Sensor _mpu_sensor;
PIDController _pid(30.0, 0.0, 0.5, 0.0);  // Kp, Ki, Kd, setpoint (horizontal)
MotorDriver _motor_driver(16, 17, 18, 19, 5, 4);  // Adjusted pins

double _current_pitch = 0.0;
double _pid_output = 0.0;

unsigned long _previous_millis = 0;
const long _interval = 250; // interval in milliseconds

// Function prototypes
void handleCommand();
void handleStatus();
void handleSetPid();

/**
 * @brief Controls the motors based on the given command
 * @param command The command string to be executed (FORWARD, BACKWARD, LEFT, RIGHT, STOP)
 * @return A confirmation message indicating the result of the command
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
    // Adjust speeds for turning left
    _motor_driver.setMotorSpeed(-150, 150);
    response += " - Turning Left";
  } else if (command == "RIGHT") {
    // Adjust speeds for turning right
    _motor_driver.setMotorSpeed(150, -150);
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
 * @brief Handles the command requests
 * @details
 * This function checks if there is a command argument in the HTTP request.
 * If there is, it calls the controlMotors() function to execute the command.
 * If there isn't, it sends a 400 response with a message indicating that no
 * command was received.
 */
void handleCommand() {
  if (server.hasArg("command")) {
    // Get the command from the request
    String command = server.arg("command");
    // Print the command to the serial monitor
    Serial.print("Command received: ");
    Serial.println(command);
    // Execute the command and get the response
    String response = controlMotors(command);
    // Send the response back to the client
    server.send(200, "text/plain", response);
  } else {
    // Send a 400 response with a message if no command was received
    server.send(400, "text/plain", "No command received");
  }
}


/**
 * @brief Handle the status request
 * @details
 * This function handles the "/status" request by sending back the current
 * pitch angle and PID output.
 */
void handleStatus() {
  String response = "Pitch Angle: " + String(_current_pitch, 2);
  response += "\nPID Output: " + String(_pid_output, 2);
  server.send(200, "text/plain", response);
}

/**
 * @brief Handle the set PID values request
 * @details
 * This function checks if there are any arguments in the HTTP request
 * with the names "Kp", "Ki", or "Kd". If there are, it calls the
 * corresponding set method on the PID controller object to update
 * the values of the PID gains.
 * If there aren't any arguments, it sends a 400 response with a message
 * indicating that no PID values were received.
 */
void handleSetPid() {
  if (server.hasArg("Kp")) {
    double kp = server.arg("Kp").toDouble();
    // Set the proportional gain
    _pid.setKp(kp);
    Serial.println("Kp set to: " + String(kp));
  }
  if (server.hasArg("Ki")) {
    double ki = server.arg("Ki").toDouble();
    // Set the integral gain
    _pid.setKi(ki);
    Serial.println("Ki set to: " + String(ki));
  }
  if (server.hasArg("Kd")) {
    double kd = server.arg("Kd").toDouble();
    // Set the derivative gain
    _pid.setKd(kd);
    Serial.println("Kd set to: " + String(kd));
  }
  // Send a confirmation message back to the client
  server.send(200, "text/plain", "PID values updated");
}

void setup() {
  Serial.begin(9600);
  // Initialize the MPU6050 sensor
  _mpu_sensor.initialize();

  // Initialize the motor driver
  _motor_driver.initialize();

  // Connect to WiFi network
  WiFi.begin(ssid, password);

  // Wait for the WiFi connection to be established
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); // Delay for 1 second before retrying
    Serial.println("Connecting to WiFi...");
  }

  // Connection successful, print the IP address
  Serial.println("Connected to WiFi");
  Serial.println("The IP address is: " + WiFi.localIP().toString());

  // Set up mDNS responder with the desired name
  if (MDNS.begin("selfbalancingrobot")) {
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error starting mDNS");
  }

  // Set up HTTP server routes
  server.on("/command", handleCommand); // Route for handling commands
  server.on("/status", handleStatus);   // Route for checking status
  server.on("/set_pid", handleSetPid);  // Route for setting PID values

  // Start the HTTP server
  server.begin();

  // Print ESP32 MAC address to serial monitor
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Set initial motor speed to zero
  _motor_driver.setMotorSpeed(0, 0);
}

/**
 * @brief The main loop of the program
 * @details
 * This function is an infinite loop that calls the server's handleClient() method
 * to handle incoming requests. It also reads the current pitch angle from the
 * MPU6050 sensor and calls the PID compute() method to calculate the PID output.
 * The pitch angle and PID output are printed to the serial monitor every 250ms.
 */
void loop() {
  // Call the server's handleClient() method to handle incoming requests
  server.handleClient();

  // Get the current time in milliseconds
  unsigned long current_millis = millis();

  // Check if the interval has passed since the last measurement
  if (current_millis - _previous_millis >= _interval) {

    // Get the current pitch angle from the MPU6050 sensor
    _current_pitch = _mpu_sensor.getPitchAngle();

    // Calculate the PID output
    _pid_output = _pid.compute(_current_pitch);

    // Print the pitch angle and PID output to the serial monitor
    Serial.print("Pitch Angle: ");
    Serial.print(_current_pitch);
    Serial.print("\tPID Output: ");
    Serial.println(_pid_output);
  }
}
