/**
 * @file gripper_control_system.ino
 * @brief Sensor-controlled DC motor gripper system with PID control
 * @author Adil Wahab Bhatti
 * @version 3.1
 * @date 2023-11-21
 * 
 * @description
 * This Arduino sketch implements a sophisticated gripper control system that uses
 * multiple sensors (ultrasonic, current, force) and a joystick for precise motor
 * control with PID feedback. The system automatically grips objects when detected
 * and provides manual control through a joystick interface.
 * 
 * @hardware
 * - Arduino Uno
 * - INA219 Current Sensor
 * - Ultrasonic Sensor (HC-SR04)
 * - Force Sensor (FSR)
 * - Servo Motor
 * - DC Motor with Driver
 * - Joystick Module (KY-023)
 * 
 * @dependencies
 * - Adafruit_INA219 Library v1.2.3+
 * - movingAvg Library v2.3.1+
 * - Servo Library (built-in)
 */

#include <Adafruit_INA219.h>
#include <movingAvg.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================

// Debug Configuration
const bool DEBUG_ENABLED = true;

// Pin Definitions
namespace Pins {
  // Servo
  const int SERVO = 11;
  
  // Motor Driver
  const int MOTOR_PWM = 3;
  const int MOTOR_DIR = 8;
  
  // Sensors
  const int FORCE_SENSOR = A0;
  const int ULTRASONIC_TRIG = 6;
  const int ULTRASONIC_ECHO = 5;
  
  // Joystick
  const int JOYSTICK_X = A2;
  const int JOYSTICK_Y = A1;
  const int JOYSTICK_BUTTON = 4;
}

// System Limits & Thresholds
namespace Limits {
  const int SERVO_MIN_ANGLE = 0;
  const int SERVO_MAX_ANGLE = 180;
  const int SERVO_START_ANGLE = 90;
  
  const int MOTOR_SPEED = 70;
  const int PWM_MIN = 0;
  const int PWM_MAX = 255;
  
  const float GRIP_DISTANCE_THRESHOLD = 12.0;  // cm
  const float JOYSTICK_DEADZONE = 0.07;        // 7%
  const float JOYSTICK_GRIP_THRESHOLD = 0.3;   // 30%
  const float LOOSENING_TIMEOUT = 9000.0;      // ms
}

// PID Controller Parameters
namespace PID {
  const double KP = 0.02;
  const double KI = 0.00;
  const double KD = 0.000000;
  const float SETPOINT = 130.0;
}

// Moving Average Buffer Sizes
namespace BufferSizes {
  const int CURRENT = 35;
  const int FORCE = 20;
  const int DISTANCE = 100;
}

// ============================================================================
// GRIPPER STATES
// ============================================================================

enum class GripperMode {
  OFF = 0,
  TIGHTENING = 1,
  TIGHTENED = 2,
  LOOSENING = -1,
  LOOSENED = -2
};

// ============================================================================
// GLOBAL OBJECTS & VARIABLES
// ============================================================================

// Hardware Objects
Adafruit_INA219 currentSensor;
Servo servoMotor;

// Moving Average Filters
movingAvg currentFilter(BufferSizes::CURRENT);
movingAvg forceFilter(BufferSizes::FORCE);
movingAvg distanceFilter(BufferSizes::DISTANCE);

// Sensor Data
struct SensorData {
  float current = 0.0;
  float avgCurrent = 0.0;
  float distance = 0.0;
  float avgDistance = 0.0;
  int force = 0;
  int avgForce = 0;
  float joystickX = 0.0;
  float joystickY = 0.0;
  bool buttonPressed = false;
};

// System State
struct SystemState {
  bool running = false;
  GripperMode gripperMode = GripperMode::OFF;
  float servoAngle = Limits::SERVO_START_ANGLE;
  
  // PID Variables
  double pidOutput = 0.0;
  double error = 0.0;
  double prevError = 0.0;
  double integral = 0.0;
  
  // Timing
  unsigned long currentTime = 0;
  unsigned long prevTime = 0;
  float deltaTime = 0.0;
  float looseningTime = 0.0;
};

SensorData sensors;
SystemState state;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Read distance from ultrasonic sensor
 * @param trigPin Trigger pin
 * @param echoPin Echo pin
 * @return Distance in centimeters
 */
float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034) / 2.0;
}

/**
 * @brief Convert joystick analog reading to normalized ratio
 * @param pin Analog pin to read
 * @return Normalized value between -1.0 and 1.0
 */
float getJoystickRatio(int pin) {
  float rawValue = analogRead(pin);
  return (rawValue * 2.0 / 1023.0) - 1.0;
}

/**
 * @brief Update all sensor readings
 */
void updateSensors() {
  // Current sensor
  sensors.current = currentSensor.getCurrent_mA();
  if (sensors.current != 0) {
    sensors.avgCurrent = currentFilter.reading(sensors.current);
  }
  
  // Distance sensor
  sensors.distance = readUltrasonicDistance(Pins::ULTRASONIC_TRIG, Pins::ULTRASONIC_ECHO);
  if (sensors.distance != 0) {
    sensors.avgDistance = distanceFilter.reading(sensors.distance);
  }
  
  // Force sensor
  sensors.force = analogRead(Pins::FORCE_SENSOR);
  sensors.avgForce = forceFilter.reading(sensors.force);
  
  // Joystick
  sensors.joystickX = getJoystickRatio(Pins::JOYSTICK_X);
  sensors.joystickY = getJoystickRatio(Pins::JOYSTICK_Y);
  sensors.buttonPressed = (digitalRead(Pins::JOYSTICK_BUTTON) == LOW);
}

/**
 * @brief Update timing variables
 */
void updateTiming() {
  state.prevTime = state.currentTime;
  state.currentTime = micros();
  state.deltaTime = (state.currentTime - state.prevTime) / 1000.0; // Convert to ms
}

/**
 * @brief Calculate PID output
 */
void calculatePID() {
  state.prevError = state.error;
  state.error = PID::SETPOINT - sensors.avgCurrent;
  
  double proportional = PID::KP * state.error;
  state.integral += PID::KI * state.error * state.deltaTime;
  double derivative = PID::KD * (state.error - state.prevError) / state.deltaTime;
  
  double pidValue = proportional + state.integral + derivative;
  state.pidOutput += pidValue;
  state.pidOutput = constrain(state.pidOutput, Limits::PWM_MIN, Limits::PWM_MAX);
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Stop the gripper motor
 */
void stopGripper() {
  analogWrite(Pins::MOTOR_PWM, 0);
}

/**
 * @brief Start gripper tightening sequence
 */
void startTightening() {
  stopGripper();
  delay(25);
  
  state.gripperMode = GripperMode::TIGHTENING;
  digitalWrite(Pins::MOTOR_DIR, HIGH); // Set direction to tighten
  
  // Gradual start
  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED * 0.5);
  delay(25);
  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED);
  delay(25);
  
  // Reset PID
  state.integral = 0;
  state.pidOutput = Limits::MOTOR_SPEED;
}

/**
 * @brief Start gripper loosening sequence
 */
void startLoosening() {
  stopGripper();
  delay(25);
  
  state.gripperMode = GripperMode::LOOSENING;
  digitalWrite(Pins::MOTOR_DIR, LOW); // Set direction to loosen
  
  // Gradual start
  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED * 0.5);
  delay(25);
  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED);
  
  state.looseningTime = 0;
}

/**
 * @brief Control gripper based on sensor inputs
 */
void controlGripper() {
  bool objectInRange = (sensors.avgDistance < Limits::GRIP_DISTANCE_THRESHOLD) && 
                      (abs(sensors.joystickX) < Limits::JOYSTICK_GRIP_THRESHOLD);
  
  if (objectInRange) {
    if (state.gripperMode != GripperMode::TIGHTENING) {
      startTightening();
    } else {
      // Continue PID control
      calculatePID();
      analogWrite(Pins::MOTOR_PWM, state.pidOutput);
      delay(10);
    }
  } else if (state.gripperMode != GripperMode::LOOSENED) {
    if (state.gripperMode != GripperMode::LOOSENING) {
      startLoosening();
    } else {
      // Continue loosening with timeout
      state.looseningTime += state.deltaTime;
      if (state.looseningTime > Limits::LOOSENING_TIMEOUT) {
        state.gripperMode = GripperMode::LOOSENED;
        stopGripper();
        state.servoAngle = Limits::SERVO_START_ANGLE;
        servoMotor.write(state.servoAngle);
      }
    }
  }
}

/**
 * @brief Control servo motor based on joystick input
 */
void controlServo() {
  if (abs(sensors.joystickY) > Limits::JOYSTICK_DEADZONE) {
    state.servoAngle += (sensors.joystickY * 0.6);
    state.servoAngle = constrain(state.servoAngle, Limits::SERVO_MIN_ANGLE, Limits::SERVO_MAX_ANGLE);
    servoMotor.write(state.servoAngle);
  }
}

/**
 * @brief Handle system on/off toggle
 */
void handleSystemToggle() {
  static bool prevButtonState = false;
  
  if (sensors.buttonPressed && !prevButtonState) {
    state.running = !state.running;
    
    // Wait for button release with debouncing
    while (digitalRead(Pins::JOYSTICK_BUTTON) == LOW) {
      delay(10);
    }
    delay(50); // Additional debounce delay
  }
  
  prevButtonState = sensors.buttonPressed;
  
  // Turn off gripper if system is disabled
  if (!state.running && state.gripperMode != GripperMode::OFF) {
    state.gripperMode = GripperMode::OFF;
    stopGripper();
  }
}

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

/**
 * @brief Print debug information to serial
 */
void printDebugInfo() {
  if (!DEBUG_ENABLED) return;
  
  Serial.print("Current: "); Serial.print(sensors.avgCurrent);
  Serial.print(" | Setpoint: "); Serial.print(PID::SETPOINT);
  Serial.print(" | Output: "); Serial.print(state.pidOutput);
  Serial.print(" | Distance: "); Serial.print(sensors.avgDistance);
  Serial.print(" | Running: "); Serial.print(state.running ? "YES" : "NO");
  Serial.print(" | Mode: "); Serial.println(static_cast<int>(state.gripperMode));
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup() {
  // Initialize serial communication for debugging
  if (DEBUG_ENABLED) {
    Serial.begin(9600);
    while (!Serial) {
      delay(1);
    }
    Serial.println("=== Gripper Control System Initializing ===");
  }
  
  // Initialize current sensor
  if (!currentSensor.begin()) {
    if (DEBUG_ENABLED) {
      Serial.println("ERROR: Failed to initialize INA219 current sensor!");
    }
  } else if (DEBUG_ENABLED) {
    Serial.println("✓ Current sensor initialized");
  }
  currentFilter.begin();
  
  // Initialize servo motor
  if (!servoMotor.attach(Pins::SERVO)) {
    if (DEBUG_ENABLED) {
      Serial.println("ERROR: Failed to initialize servo motor!");
    }
  } else if (DEBUG_ENABLED) {
    Serial.println("✓ Servo motor initialized");
  }
  
  // Set servo to starting position
  servoMotor.write(state.servoAngle);
  while (servoMotor.read() != state.servoAngle) {
    delay(10);
  }
  
  // Initialize motor driver pins
  pinMode(Pins::MOTOR_PWM, OUTPUT);
  pinMode(Pins::MOTOR_DIR, OUTPUT);
  
  // Initialize sensor pins
  pinMode(Pins::FORCE_SENSOR, INPUT);
  forceFilter.begin();
  
  pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
  pinMode(Pins::ULTRASONIC_ECHO, INPUT);
  distanceFilter.begin();
  
  // Initialize joystick pins
  pinMode(Pins::JOYSTICK_X, INPUT);
  pinMode(Pins::JOYSTICK_Y, INPUT);
  pinMode(Pins::JOYSTICK_BUTTON, INPUT_PULLUP);
  
  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  if (DEBUG_ENABLED) {
    Serial.println("=== System Ready ===");
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Update all sensors and timing
  updateSensors();
  updateTiming();
  
  // Handle system toggle
  handleSystemToggle();
  
  // Main control logic (only when system is running)
  if (state.running) {
    controlGripper();
    controlServo();
  }
  
  // Debug output
  printDebugInfo();
  
  // Small delay to prevent overwhelming the system
  delay(10);
}
