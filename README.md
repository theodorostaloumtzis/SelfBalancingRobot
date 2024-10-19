Here's the improved version of your README:

---

# Self-Balancing Robot Control System

This repository contains the firmware for a **self-balancing robot** powered by an **ESP32** and an **MPU6050 sensor**. The robot is controlled via a Flutter app (see [robot_control_app](https://github.com/theodorostaloumtzis/robot_control_app)) using WiFi to send movement commands and dynamically adjust PID parameters.

## Features

The **Self-Balancing Robot** is designed to autonomously maintain its balance while being remotely controlled. Utilizing an **ESP32 microcontroller**, an **MPU6050 sensor**, and a **PID control algorithm**, this robot demonstrates advanced robotics concepts with practical applications in **Internet of Things (IoT)** technology. The user can control the robot through a **Flutter mobile application**, which allows for easy direction control and PID tuning.

### Key Features

- **WiFi-Controlled**: Users can control the robot wirelessly within WiFi range.
- **Autonomous Balancing**: Real-time data from the MPU6050 sensor helps the robot maintain balance automatically.
- **Directional Movement**: The robot can move forward, backward, left, and right, commanded by the mobile app.
- **Mobile App Integration**: A Flutter app provides a simple interface to control the robot and tune the PID values.
- **Real-Time Sensor Feedback**: The robot continuously monitors its pitch angle, adjusting movements dynamically.

#### Hardware

- **ESP32** for control and WiFi communication
- **MPU6050** for measuring the pitch and roll angles
- **L298N Motor Driver** for controlling the motors
- **DC Motors** for movement
- **12V Power Supply** for powering the motors and board
- **Switch** to control power flow

Refer to the connection diagram for wiring the components to match the pin assignments in the code.

![Circuit](images/Connections_Diagram.png)


#### Software

- **Firmware**: Program the ESP32 using the Arduino IDE or PlatformIO with the provided code.
- **Flutter App**: The robot is controlled via the [robot_control_app](https://github.com/theodorostaloumtzis/robot_control_app) over WiFi.

## Features in Detail

- **Self-Balancing**: The robot balances using data from the MPU6050 sensor, managed by a PID control system.
- **WiFi Communication**: Commands and data flow between the ESP32 and the app over WiFi.
- **Dynamic PID Tuning**: The user can adjust PID parameters (Kp, Ki, Kd) from the app to fine-tune robot stability.
- **Real-Time Updates**: The ESP32 provides real-time status updates, including pitch angle and PID output.

## HTTP Endpoints

The ESP32 exposes the following HTTP endpoints for controlling the robot and adjusting its settings:

- `/command?command=<ACTION>`: Sends movement commands. Valid actions: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `STOP`.
- `/status`: Returns the current pitch angle and PID controller output in real-time.
- `/set_pid?Kp=<value>&Ki=<value>&Kd=<value>`: Updates the PID controller's Kp, Ki, and Kd values.

### Example Usage

- **Movement Commands**:
  To move the robot forward, send:
  ```bash
  http://<robot-ip>/command?command=FORWARD
  ```
  
- **Set PID Values**:
  Dynamically update the PID parameters:
  ```bash
  http://<robot-ip>/set_pid?Kp=1.0&Ki=0.1&Kd=0.05
  ```

- **Fetch Robot Status**:
  Get the current pitch angle and PID output:
  ```bash
  http://<robot-ip>/status
  ```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Author

- **Theodoros Taloumtzis** - [GitHub](https://github.com/theodorostaloumtzis)

---

This version adds clarity and removes redundant information, while keeping all the essential technical details.