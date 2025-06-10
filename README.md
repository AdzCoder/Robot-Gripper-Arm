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
- **Power:** 12 V, 1 A (12 W) plug-in power supply ×1  
- **Weight Capacity:** 10 g – 500 g  
- **Object Size Range:** 15 × 10 × 10 mm – 60 × 80 × 150 mm  

## Installation

1. **Clone this repository**  
    ```bash
    git clone <repository-url>
    ```

2. **Open the project**  
    Open `src/Robotic_Hand.ino` using the [Arduino IDE](https://www.arduino.cc/en/software).

3. **Install required libraries**  
   Install the following via the Arduino Library Manager or GitHub:

   - **[Adafruit INA219](https://github.com/adafruit/Adafruit_INA219)** — v1.2.3, MIT License  
     For current sensing with the INA219 sensor.  
   - **[movingAvg](https://github.com/JChristensen/movingAvg)** — v2.3.2, GPL-3.0  
     For calculating moving averages of sensor readings.  
   - **Servo** — included with Arduino IDE (LGPL-2.1)  
     For controlling servo motors.

4. **Upload to Arduino Uno**  
   Once libraries are installed, connect your board and upload the sketch.

## Usage

1. Power on the device.  
2. Use the joystick to position the gripper.  
3. Press the joystick button to activate autonomous gripping.  
4. Push the joystick forward/backwards for emergency release.

## Project Team

- J4  
- University of Warwick, School of Engineering  
- ES2C6 Electromechanical System Design (2023)

## Project Status

This project was completed as part of coursework for the University of Warwick (ES2C6 module, 2023).  
It is no longer actively maintained, but the code and documentation are provided for reference and learning purposes.

## License

MIT License — see the [LICENSE](LICENSE) file for details.
