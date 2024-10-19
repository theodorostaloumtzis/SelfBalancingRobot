Here’s the enhanced README file written in Markdown format:

# Self-Balancing Robot

## Project Overview

The **Self-Balancing Robot** is an innovative project designed to create a mobile robot capable of maintaining its balance autonomously while being controlled remotely. Utilizing an **ESP32 microcontroller**, an **MPU6050 sensor**, and a **PID control algorithm**, this robot not only demonstrates advanced robotics concepts but also provides a practical application of Internet of Things (IoT) technologies. Users can command the robot to move in various directions through a user-friendly Flutter mobile application.

## Key Features

- **WiFi-Controlled**: Seamless connectivity allows users to control the robot from anywhere within range.
- **Autonomous Balancing**: The robot utilizes real-time data from the MPU6050 to maintain an upright position, ensuring stability during movement.
- **Directional Movement**: Move the robot forward, backward, left, or right with simple commands from the app.
- **Mobile App Integration**: A dedicated Flutter application offers an intuitive interface for controlling the robot.
- **Real-Time Sensor Feedback**: Continuously monitors pitch and roll angles to adjust movements dynamically.

## Hardware Setup

The **Self-Balancing Robot** requires the following hardware:

- ESP32 Development Board
- MPU6050 Sensor
- L298N Motor Driver
- DC Motors
- Power Supply 12V 
- Switch

## Diagram Circuit

![Circuit](images/Connections_Diagram.png)

Here you can see the connections based on the code pins assgnments.

## Getting Started

### Prerequisites

Before you begin, ensure you have the following:

- An **ESP32** development board
- An **MPU6050** sensor
- An **L298N** motor driver
- DC motors
- A compatible mobile device with the Flutter app installed
- Arduino IDE for uploading code to the ESP32

### Installation Steps

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/theodorostaloumtzis/SelfBalancingRobot.git
   cd SelfBalancingRobot
   ```

2. **Install Necessary Libraries**:
   Open the Arduino IDE and install the following libraries:
   - **MPU6050**: For interfacing with the MPU6050 sensor.
   - **WiFi**: For enabling WiFi connectivity on the ESP32.

3. **Upload the Code**:
   Open the Arduino project file in the Arduino IDE, connect the ESP32 to your computer, and upload the code.

4. **Hardware Setup**:
   Connect the components as illustrated in the provided circuit diagram. Ensure all connections are secure.

5. **Flutter App Configuration**:
   - Set up the Flutter app to establish Bluetooth communication.
   - Follow the setup instructions in the app’s documentation.

### Usage Instructions

1. **Power On the Robot**: Turn on the robot and ensure it is on a stable surface.
2. **Connect to WiFi**: Use your mobile device to connect to the robot's WiFi network.
3. **Open the Flutter App**: Launch the app and connect to the robot.
4. **Control the Robot**: Use the app's controls to move the robot while it self-balances.

## Code Structure

- **`src/`**: Contains the main firmware for the self-balancing robot.
- **`README.md`**: Project documentation providing an overview and instructions.

## Contributing

We welcome contributions! If you have suggestions for improvements or features, please fork the repository and submit a pull request. Your contributions help make this project better.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project was developed by the [Theodorostaloumtzis](https://github.com/theodorostaloumtzis) team.

