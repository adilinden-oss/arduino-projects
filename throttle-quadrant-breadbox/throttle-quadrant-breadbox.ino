/*
  throttle-quadrant-breadbox.ino
 
  Code to support an experimental throttle quadrant and additional switches
  to use with Microsoft Flight Simulator 2020. In particular it features the
  following input devices:

  3 Potentimeters for throttle, prop and mixture
  1 Rotary encoder for elevator trim
  1 push button for reverse thrust
  1 SPST switch for gear up - down
  1 SPDT switch for flaps up - t/o - ldg 

  It requires the Joystick library from:
  https://github.com/MHeironimus/ArduinoJoystickLibrary

  My prototype was built using the *uino-32u4 board I developed back in
  2011. It is a ATmega 34U4 desgin and thus this should work equally we
  with the Arduino Leonardo and any of its derivatives. 
  
  The board details can be found at: 
  https://github.com/adilinden-oss/uino-32u4.

  The supporting board manager files can be found at:
  https://github.com/adilinden-oss/uino-arduino.
*/

#include "Joystick.h"

// Create Joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
  2, 0,                     // Button count, Hat Switch count
  false, false, false,      // X, Y, Z axis
  true, true, false,        // Rx, Ry, Rz rotations
  false, true,              // Rudder, throttle
  false, false, false);     // Accelerator, brake, steering

// Debug mode
bool debug = true;          // === IMPORTANT ===
unsigned int debugEncClicks = 0;

// Loop timings
const unsigned long debugPeriod = 200;
const unsigned long pressPeriod = 100;
unsigned long debugTimer = 0;
unsigned long buttonUpTimer = 0;
unsigned long buttonDownTimer = 0;

// HW pin used
const int sensorPin1 = A0;    // select the input pin for the potentiometer
const int sensorPin2 = A1;    // select the input pin for the potentiometer
const int sensorPin3 = A2;    // select the input pin for the potentiometer
const int encPinA = A4;
const int encPinB = A5;

// Pot values
int lastSensorVal1 = 0;
int lastSensorVal2 = 0;
int lastSensorVal3 = 0;

// Rotary encoder values
int lastEncValA = 0;
int lastEncValB = 0;

// Button press
bool isPressedUp = false;
bool isPressedDown = false;

void setup() {
  // Pins
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(encPinA, INPUT);
  pinMode(encPinB, INPUT);

  // Joystick
  Joystick.setThrottleRange(0, 1023);
  Joystick.setRxAxisRange(0, 1023);
  Joystick.setRyAxisRange(0, 1023);

  // Initialize joystick
  Joystick.begin();

  if (debug) { Serial.begin(9600); }
}

void loop() {
  // Handle pots
  int readSensor = 0;

  readSensor = correctTaper(analogRead(sensorPin1));
  if (readSensor != lastSensorVal1) {
    Joystick.setThrottle(readSensor);
    lastSensorVal1 = readSensor;
  }
  readSensor = correctTaper(analogRead(sensorPin2));
  if (readSensor != lastSensorVal2) {
    Joystick.setRxAxis(readSensor);
    lastSensorVal2 = readSensor;
  }
  readSensor = correctTaper(analogRead(sensorPin3));
  if (readSensor != lastSensorVal3) {
    Joystick.setRyAxis(readSensor);
    lastSensorVal3 = readSensor;
  }

  // Handle encoder
  int readEncA = digitalRead(encPinA);
  int readEncB = digitalRead(encPinB);

  if (lastEncValA && !readEncA) {
    // A dropped
    if (readEncB) {
      // A drop & B high = down
      buttonDown();
    } else {
      // A drop & B low = up
      buttonUp();      
    }
  }
  if (!lastEncValA && readEncA) {
    // A rose
    if (readEncB) {
      // A rise & B high = up
      buttonUp();
    } else {
      // A rise & B low = down
      buttonDown();      
    }
  }
  if (lastEncValB && !readEncB) {
    // B dropped
    if (readEncA) {
      // B drop & A high = up
      buttonUp();
    } else {
      // B drop & A low = down
      buttonDown();      
    }
  }
  if (!lastEncValB && readEncB) {
    // B rose
    if (readEncA) {
      // B rise & A high = down
      buttonDown();
    } else {
      // B rise & A low = up
      buttonUp();      
    }
  }
  lastEncValA = digitalRead(encPinA);
  lastEncValB = digitalRead(encPinB);

  if (isPressedUp && (millis() - buttonUpTimer > pressPeriod)) {
    Joystick.setButton(0,0);
    isPressedUp = false;
  }
  if (isPressedDown && (millis() - buttonDownTimer > pressPeriod)) {
    Joystick.setButton(1,0);
    isPressedDown = false;
  }

  // Debug printing
  if (debug) {
    if (millis() - debugTimer > debugPeriod) {
      char buffer[60];
      sprintf(buffer, "Debug: Pots: %4d  %4d  %4d -- Enc: %d  %d Clicks: %u", 
        lastSensorVal1, lastSensorVal2, lastSensorVal3, 
        lastEncValA, lastEncValB, debugEncClicks);
      Serial.println(buffer);
      debugTimer = millis();
    }
  }

  delay(5);
}

void buttonUp() {
  if (!isPressedUp) {
    Joystick.setButton(0,1);
    isPressedUp = true;
    buttonUpTimer = millis();
    debugEncClicks++;
  }
}

void buttonDown() {
  if (!isPressedUp) {
    Joystick.setButton(1,1);
    isPressedDown = true;
    buttonDownTimer = millis();
    debugEncClicks--;
  }
}

// We have shitty pots, this corrects the B5 logarithmic taper to some
// approimation of linear.
int correctTaper(int in) {
  int out;
  
  if (in < 58) {
    out = in * 5;
  }
  else if (in < 512) {
    out = (in - 57) / 2 + 285;
  }
  else if (in < 967) {
    out = 1023 - ((1023 - in - 57) / 2 + 285);
  }
  else if (in < 1021) {
    out = 1023 - ((1023 - in) *5);
  }
  else {
    out = 1023;
  }

  return out;
}
