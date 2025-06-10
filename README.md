# Robot Gripper Arm Project

## Overview

An electromechanical gripper arm prototype designed to assist individuals with limited mobility in performing daily tasks.  
The device features autonomous and adaptive gripping capabilities using force and distance feedback, with an intuitive joystick interface and built-in safety mechanisms.

## Features

- Autonomous grip control using force and distance sensors
- Ergonomic bike-handle grip design
- PID control for consistent grip strength
- Joystick interface for intuitive control
- Safety features including emergency release

## Technical Specifications

- **Microcontroller:** Arduino Uno
- **Sensors:**
  - INA219 Current Sensor
  - HC-SR04 Ultrasonic Distance Sensor
  - Force Sensing Resistor
- **Actuators:**
  - DC Motor for gripping
  - Servo Motor for rotation
- **Power:** 12V, 1A, 12W Plug-in Power Supply ×1
- **Weight Capacity:** 10g to 500g
- **Object Size Range:** 15×10×10mm to 60×80×150mm

## Installation

1. **Clone this repository**

    ```bash
    git clone <repository-url>
    ```

2. **Open the project**

    Open `src/Robotic_Hand.ino` using the [Arduino IDE](https://www.arduino.cc/en/software).

3. **Install required libraries**

    Make sure the following libraries are installed (via the Arduino Library Manager or by downloading them from GitHub):

    - **Adafruit INA219** — v1.2.3  
      For current sensing with the INA219 sensor.

    - **movingAvg** — v2.3.1  
      For calculating moving averages of sensor readings.  
      *License: GNU GPL v3.0*

    - **Servo** — built-in or available via Library Manager  
      For controlling servo motors.  
      *Copyright © 2009 Michael Margolis*

4. **Upload to Arduino Uno**

    Once all libraries are installed, connect your Arduino Uno and upload the project.

## Usage

1. Power on the device.
2. Use the joystick to position the gripper.
3. Press the joystick button to activate autonomous gripping.
4. Move joystick forward/back for emergency release.

## Project Team

- J4  
- University of Warwick, School of Engineering  
- ES2C6 Electromechanical System Design (2023)

## Project Status

This project was completed as part of coursework for the University of Warwick (ES2C6 module, 2023).  
It is no longer actively maintained, but the code and documentation are provided for reference and learning purposes.

## License

MIT License — see the [LICENSE](LICENSE) file for details.
