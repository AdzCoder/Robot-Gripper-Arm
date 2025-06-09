/*
  File type: Arduino Sketch file
  Author: Adil Wahab Bhatti
  Board: Arduino Uno
  Function: Utilise sensor measurements to control and operate DC motors
  Version: 3.1 (Final)
  Date: 21/11/2023
*/

// Moving Averages - Library
  #include <movingAvg.h> // Arduino Moving Average Library (v2.3.1)
    // https://github.com/JChristensen/movingAvg
    // Copyright (C) 2018 by Jack Christensen and licensed under
    // GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

// Current Sensor (INA219)- Library, Objects, Constant, and Variables
  #include <Adafruit_INA219.h> // Adafruit INA219 Library (v1.2.3)
    // https://github.com/adafruit/Adafruit_INA219
    // Written by Bryan Siepert and Kevin "KTOWN" Townsend for Adafruit Industries
    // BSD license

  Adafruit_INA219 ina219;
  movingAvg currentArray(35); // array to hold last 35 values of current readings

  float current; // current read from the sensor
  float avgCurrent; // average current calculated from range

// Servo Motor - Library, Object, Constants, and Variable
  #include <Servo.h> // Interrupt driven Servo library for Arduino using 16 bit timers (v2)
    // Copyright (c) 2009 Michael Margolis.  All right reserved.

  Servo servoRoll; // servo controller

  const int servoPin = 11; // digital pin 11 will be used for PWM servo output
  const int minAngle = 0; // angle cannot go below 0
  const int maxAngle = 180; // angle cannot exceed 180
  float angle = 90; // servo starting position set to 90

// Motordriver - Constants and Variable
  const int pwmPin = 3; // digital pin 3 will be used for PWM motor driver output
  const int dirPin = 8; // digital pin 8 will be used for directional output

  int motorSpeed = 70; // motor speed set to 70
  int gripMode = 0; // mode of the gripper
    // 0 = off
    // 1 = tighting
    // 2 = tightened
    // -1 = loosening
    // -2 = loosened
 
// Force sensor - Object, Constant and Variables
  movingAvg forceArray(20); // array to hold last 20 values of force readings

  const int forcePin = A0; // analogue pin A0 will be used for force sensor input
  int force; // value read from force pin; 0 to 1023
  int avgForce; // average force calculayed from range

// Ultrasonic Sensor - Object, Constants, Variable and Function
  movingAvg distArray(100); // array to hold last 100 values of distance readings

  const int trigPin = 6; // digital pin 6 will be used for transmitter output
  const int echoPin = 5; // digital pin 5 will be used for receiver input
  float dist; // distance calculated from the sensor
  float avgDist; // average current calculated from range

  float readDistance(int trigPin, int echoPin) { // reads the distance using the ultrasonic sensor
    // Set the trig pin to LOW 
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sets the trigPin to HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH); // reads the echoPin, returns the sound wave travel time in microseconds
    float distance = duration * 0.034 / 2; // calculate the distance
    return distance; // return the distance
  }

// Joystick Module (Module KY023) - Constants, Variables, and Function
  const int xPin = A2; // analogue pin A2 will be used for X input
  const int yPin = A1; // analogue pin A1 will be used for y input
  const int bPin = 4; // digital pin 4 will be used for push button input
  float xVal, yVal; // voltage ratios read from x and y pins; -1 to 1
  int bVal; // voltage read from button pin
  bool running = false; // true when running, false when not running

  float getJoystickRatio(int pin) { // reads voltage from joystick pin and converts to a ratio
    float val = analogRead(pin); // voltage read from x and y pins; 0 to 1023
    float ratio = (val*2/1023) - 1; // convert voltage to ratio
    // float ratio = map(val, 0, 1023, -1000, 1000) / 1000
    return ratio; // return the ratio
  }

// Serial for debugging - Constant
  const bool debug = true; // set to true when debugging

// Proportional–integral–derivative controller - Constants and Variables 
  // Define PID parameters
  const double Kp = 0.02; // Proportional gain
  const double Ki = 0.00; // Integral gain
  const double Kd = 0.000000; // Derivative gain
  double P, I, D, PID;

  // Time Variables
  unsigned long t = 0, prevT = 0; // time stamps in microseconds
  float dt = 0; // elapsed time between loops in ms
  float looseT; // elapsed time while loosening in ms

  // Define setpoint and initial parameters
  const int minOutput = 0; // output cannot go below 0
  const int maxOutput = 255; // output cannot exceed 255
  double output, error, prevError;
  float setPoint = 130; // Setpoint is set to 200

// Initialisation
  void setup() {
    // initialising the current sensor
    bool inaCheck = ina219.begin(); // store output for debugging
    currentArray.begin(); // setup the current array 
  
    // initialising the servo motor
    bool servoCheck = servoRoll.attach(servoPin); // store output for debugging
    while (servoRoll.read() != angle) { // wait for servo to set to starting angle
      servoRoll.write(angle); // set servo to starting angle
    }
    
    // initialising the motor driver
    pinMode(pwmPin,OUTPUT);  // we have to set PWM pin as output
    pinMode(dirPin,OUTPUT);  // Direction pin pins are also set as output
    
    // initialising the force sensor
    pinMode(forcePin, INPUT); // sets the force pin as an input 
    forceArray.begin(); // setup the force array

    // initialising the ultrasonic sensor
    pinMode(trigPin, OUTPUT); // sets the trigPin as an output  
    pinMode(echoPin, INPUT); // sets the echoPin as an input
    distArray.begin(); // setup the distance array

    // initialising the joystick button
    pinMode(xPin, INPUT); // sets the xPin as an input  
    pinMode(yPin, INPUT); // sets the yPin as an output 
    pinMode(bPin, INPUT_PULLUP); // enable the internal pull-up resistor

    // setup board led
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off

    // setup debugger
    if (debug) { // if debugger turned on
      Serial.begin(9600); // setup serial communication for debug
      while (!Serial) { // wait for serial comunication to open
        delay(1); // will pause the Uno Microcontroller until serial console opens
      } 
      
      if (!inaCheck) { // if fails to initialize the INA219 chip
        Serial.println("Failed to connect to INA219 chip");
      }

      if (!servoCheck) { // if fails to initialize the servo motor
        Serial.println("Failed to connect to servo motor");
      }
    }
  }

// Runtime
  void loop() {
    // Code for reading sensors
    current = ina219.getCurrent_mA(); // gets current in mA from current sensor
    if (current); { // if current reading is not zero
      avgCurrent = currentArray.reading(current); // adds new reading to array and stores the new average
    }
    
    dist = readDistance(trigPin, echoPin); // read distance using the ultrasonic sensor
    if (dist); { // if distance reading is not zero
      avgDist = distArray.reading(dist); // adds new reading to array and stores the new average
    }
  
    force = analogRead(forcePin); // read voltage from output of force sensor
    avgForce = forceArray.reading(force); // adds new reading to array and stores the new average

    xVal = getJoystickRatio(xPin); // reads ratio from joystick in x-axis
    yVal = getJoystickRatio(yPin); // reads ratio from joystick in y-axis
    bVal = digitalRead(bPin); // reads the voltage from push button

    // Update elapsed time
    prevT = t; // save the previous time stamp
    t = micros(); // read the current time stamp
    dt = (t - prevT) / 1000; // calculate difference and convert microseconds to milliseconds

    // Update error values
    prevError = error;
    error = setPoint - avgCurrent;

    // Calculate P, I, D terms
    P = Kp * error;
    I += Ki * error * dt;
    D = Kd * (error - prevError) / dt;
    PID = P + I + D;

    // Code for switching the program on or off
    if (bVal == LOW) { // if button is pressed
      running = !running; // flip state of the program
      // digitalWrite(LED_BUILTIN, false); // flip the state of the led
      do { bVal = digitalRead(bPin); // reads the voltage from push button
      } while (bVal == LOW); // hold program until button released
      delay(50); // delay in milliseconds
    }

    // Main Section
    if (running) { // if program turned on
      // Code for controlling the gripper
      if (dist < 12 && abs(xVal) < 0.3) { // if the object is within 5cm from sensor and joystick is not pulled back more than 30%
        if (gripMode != 1) { // if gripper not already tighting   
          // Pause gripper
          analogWrite(pwmPin, 0);
          delay(25); // delay in milliseconds
          
          // Set gripper to tighten
          gripMode = 1; // set mode to tightening
          digitalWrite(dirPin, HIGH); // set direction to tight

          // Begin tightening gripper
          analogWrite(pwmPin, motorSpeed*0.5); // turn motor on half speed
          delay(25); // delay in milliseconds
          analogWrite(pwmPin, motorSpeed); // turn motor on full speed
          delay(25); // delay in milliseconds

          // Reset integral and output for PID
          I = 0;
          output = motorSpeed;
        
        } else { // if gripper has started to grip
          output += PID; // incremental PID calculation
          output = constrain(output, minOutput, maxOutput); // keep output within range
          analogWrite(pwmPin, output); // turn motor with PID output
          delay(10); // delay in milliseconds
        }

      } else if (gripMode != -2) { // if there is no object within the grips and gripper not already loosened
        if (gripMode != -1) {
          // Pause gripper
          analogWrite(pwmPin,0);
          delay(25); // delay in milliseconds
          
          // Set gripper to loose
          gripMode = -1; // set mode to loosening
          digitalWrite(dirPin, LOW); // set direction to loose

          // Begin loosening gripper
          analogWrite(pwmPin, motorSpeed*0.5); // turn motor on
          delay(25); // delay in milliseconds
          analogWrite(pwmPin, motorSpeed); // turn motor on 
          
          // Start tracking time of loosening
          looseT = 0;

        } else {
          looseT += dt;
          if (looseT > 9000) { // if gripper has been loosening for 9s
            gripMode = -2; // set mode to loosened  
            analogWrite(pwmPin, 0); // turn motor off
            angle = 90; // revert angle of servo to starting position
            servoRoll.write(angle); // set new angle
          }
        }        
      }
      
      // Code to control the servo motor
      if (abs(yVal) > 0.07) { // if joystick is deflected left or right by more than 7%
        angle += (yVal*0.6); // add joystic deflection ratio * 0.5 to angle
        angle = constrain(angle, minAngle, maxAngle); // keep angle within range
        servoRoll.write(angle); // set new angle
      }
    } else if (gripMode) { // if program turned off and gripper not turned off
      gripMode = 0; // set mode to off
      analogWrite(pwmPin, 0); // turn motor off
    }

    // Debugger
    if (debug) { // if debugger turned on
      Serial.println(""); Serial.print(avgCurrent);
      Serial.print(", "); Serial.print(setPoint);
      Serial.print(", "); Serial.print(output);
      // Serial.print(", "); Serial.print(force);
      // delay(100);
    }
  }