# Robot Gripper Arm Project

![Arduino](https://img.shields.io/badge/Arduino-Uno-blue?style=flat-square)
![Licence](https://img.shields.io/badge/Licence-MIT-orange?style=flat-square)
![University](https://img.shields.io/badge/University-Warwick-green?style=flat-square)
![Status](https://img.shields.io/badge/Status-Educational-lightgrey?style=flat-square)
![Domain](https://img.shields.io/badge/Domain-Robotics-red?style=flat-square)

## Overview

An electromechanical gripper arm prototype designed to assist individuals with limited mobility in performing daily tasks. The device features autonomous and adaptive gripping capabilities using force and distance feedback, with an intuitive joystick interface and built-in safety mechanisms.

### Key Applications
- Assistive technology for individuals with mobility impairments
- Educational robotics and mechatronics demonstrations
- Research platform for adaptive gripping algorithms

## Features

- 🤖 **Autonomous Grip Control** — Intelligent gripping using force and distance sensor feedback
- 🎯 **Ergonomic Design** — Bike-handle grip design optimised for accessibility
- ⚙️ **PID Control System** — Consistent grip strength regulation for delicate objects
- 🕹️ **Intuitive Interface** — Joystick control for precise positioning and operation
- 🛡️ **Safety Mechanisms** — Emergency release and force limiting protection
- 📊 **Real-time Feedback** — Continuous monitoring of grip force and object detection

## Technical Specifications

### Hardware Components
- **Microcontroller:** Arduino Uno R3
- **Power Supply:** 12V, 1A (12W) plug-in adapter
- **Weight Capacity:** 10g – 500g
- **Object Dimensions:** 15×10×10mm – 60×80×150mm

### Sensors & Actuators
| Component | Model | Purpose |
|-----------|--------|---------|
| **Current Sensor** | INA219 | Motor current monitoring and force feedback |
| **Distance Sensor** | HC-SR04 Ultrasonic | Object detection and positioning |
| **Force Sensor** | Force Sensing Resistor (FSR) | Direct grip pressure measurement |
| **Drive Motor** | DC Geared Motor | Primary gripping mechanism |
| **Positioning Motor** | Servo Motor | Gripper rotation and alignment |

### Control System
- **Algorithm:** PID feedback control
- **Response Time:** <100ms sensor-to-actuator
- **Precision:** ±2mm positioning accuracy
- **Safety Limits:** Configurable force thresholds

## Quick Start

### Prerequisites
- **Arduino IDE** (version 1.8.0 or later)
- **USB Cable** (Type A to Type B)
- **12V Power Supply** (included specifications)

### Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/AdzCoder/robot-gripper-arm.git
   cd robot-gripper-arm
   ```

2. **Install Required Libraries**
   
   Install the following libraries via Arduino Library Manager (`Tools > Manage Libraries...`):
   
   | Library | Version | Licence | Purpose |
   |---------|---------|---------|---------|
   | [Adafruit INA219](https://github.com/adafruit/Adafruit_INA219) | v1.2.3+ | MIT | Current sensing and power monitoring |
   | [movingAvg](https://github.com/JChristensen/movingAvg) | v2.3.2+ | GPL-3.0 | Signal filtering and noise reduction |
   | Servo | Built-in | LGPL-2.1 | Servo motor control |

3. **Upload the Code**
   ```arduino
   // Open src/Robotic_Hand.ino in Arduino IDE
   // Select Tools > Board > Arduino Uno
   // Select appropriate COM port
   // Click Upload button
   ```

### Hardware Setup

1. **Connect Power Supply** — Ensure 12V adapter is properly connected
2. **Verify Connections** — Check all sensor and motor wiring per circuit diagram
3. **Calibrate Sensors** — Run initial calibration routine (see User Manual)

## Usage Guide

### Basic Operation
1. **Power On** — Switch on main power and wait for initialisation LED
2. **Position Gripper** — Use joystick X/Y axes for precise positioning
3. **Activate Grip** — Press joystick button to engage autonomous gripping
4. **Release Object** — Push joystick forward for controlled release

### Advanced Features
- **Force Adjustment** — Modify grip strength via potentiometer
- **Emergency Stop** — Pull joystick backwards for immediate release
- **Calibration Mode** — Hold button during startup for sensor recalibration

### Troubleshooting
- **No Response:** Check power connections and Arduino USB link
- **Weak Grip:** Verify motor current readings and force sensor calibration
- **Positioning Issues:** Recalibrate distance sensor and check for obstructions

## Documentation

- 📖 **[User Manual](docs/user-manual.pdf)** — Complete operating instructions
- 🔧 **[Assembly Guide](docs/assembly-guide.pdf)** — Detailed construction steps  
- ⚡ **[Circuit Diagrams](docs/schematics/)** — Electrical connection diagrams
- 🧪 **[Test Results](docs/testing-report.pdf)** — Performance evaluation data

## Project Information

**Development Team:** Group J4  
**Institution:** University of Warwick, School of Engineering  
**Module:** [ES2C6: Electromechanical System Design (2023/24)](https://courses.warwick.ac.uk/modules/2023/ES2C6-15)

**Project Objectives:**
- Design and implement an assistive robotic device
- Integrate multiple sensor systems for autonomous operation
- Develop safety-critical control algorithms
- Create accessible human-machine interfaces

## Performance Metrics

- ✅ **Grip Success Rate:** 94.2% across tested object range
- ✅ **Response Time:** 85ms average sensor-to-action
- ✅ **Power Efficiency:** 8.5W average consumption
- ✅ **Safety Tests:** 100% emergency release functionality

## Future Enhancements

Potential improvements identified during development:
- **Machine Learning Integration** — Adaptive grip patterns based on object recognition
- **Wireless Control** — Bluetooth or Wi-Fi interface for remote operation
- **Multi-DOF Movement** — Additional servo motors for enhanced positioning
- **Visual Feedback** — Camera integration for improved object detection

## Contributing

This is an educational project that has been completed. However, if you're using this code for your own research or studies:

1. Fork the repository for your modifications  
2. Document any significant changes or improvements
3. Consider sharing results with the academic community
4. Respect the original licensing terms

## Project Status

**Status:** Completed (Academic Year 2023/24)  
**Maintenance:** No longer actively maintained  
**Usage:** Available for educational and research purposes

This project represents a successful completion of the ES2C6 coursework requirements and demonstrates practical application of mechatronic principles in assistive technology.

## Licence

MIT Licence — see the [LICENCE](LICENSE) file for details.

---

*This project was developed as part of academic coursework at the University of Warwick. For technical questions or educational collaboration, please contact through appropriate academic channels.*
