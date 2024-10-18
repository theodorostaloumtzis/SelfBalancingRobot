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

void handleCommand() {
  if (server.hasArg("command")) {
    String command = server.arg("command");
    Serial.print("Command received: ");
    Serial.println(command);
    String response = controlMotors(command);
    server.send(200, "text/plain", response);
  } else {
    server.send(400, "text/plain", "No command received");
  }
}

void handleStatus() {
  String response = "Pitch Angle: " + String(_current_pitch, 2);
  response += "\nPID Output: " + String(_pid_output, 2);
  server.send(200, "text/plain", response);
}

void handleSetPid() {
  if (server.hasArg("Kp")) {
    double kp = server.arg("Kp").toDouble();
    _pid.setKp(kp);
    Serial.println("Kp set to: " + String(kp));
  }
  if (server.hasArg("Ki")) {
    double ki = server.arg("Ki").toDouble();
    _pid.setKi(ki);
    Serial.println("Ki set to: " + String(ki));
  }
  if (server.hasArg("Kd")) {
    double kd = server.arg("Kd").toDouble();
    _pid.setKd(kd);
    Serial.println("Kd set to: " + String(kd));
  }
  server.send(200, "text/plain", "PID values updated");
}

void setup() {
  Serial.begin(9600);
  
  _mpu_sensor.initialize();
  _motor_driver.initialize();

  WiFi.begin(ssid, password);

  // Wait for the connection to be established
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Print the IP address to the serial monitor
  Serial.println("Connected to WiFi");
  Serial.println("The IP address is: " + WiFi.localIP().toString());

  // Set up mDNS
  if (MDNS.begin("selfbalancingrobot")) { // Use the desired mDNS name
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error starting mDNS");
  }

  server.on("/command", handleCommand);
  server.on("/status", handleStatus);
  server.on("/set_pid", handleSetPid);

  server.begin();
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  _motor_driver.setMotorSpeed(0, 0);
}

void loop() {
  server.handleClient();
  unsigned long current_millis = millis();
  if (current_millis - _previous_millis >= _interval) {
    _current_pitch = _mpu_sensor.getPitchAngle();
    _pid_output = _pid.compute(_current_pitch);
    Serial.print("Pitch Angle: ");
    Serial.print(_current_pitch);
    Serial.print("\tPID Output: ");
    Serial.println(_pid_output);
  }
}
