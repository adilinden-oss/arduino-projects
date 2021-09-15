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
const bool debug = false;          // === IMPORTANT ===
unsigned int debugEncClicks = 0;

// Loop timings
const unsigned long debugPeriod = 200;
const unsigned long encTriggerPeriod = 100;
unsigned long debugTimer = 0;
unsigned long encUpTimer = 0;
unsigned long encDownTimer = 0;

// HW pin used
const int potPin1 = A0;    // select the input pin for the potentiometer
const int potPin2 = A1;    // select the input pin for the potentiometer
const int potPin3 = A2;    // select the input pin for the potentiometer
const int encPinA = A4;
const int encPinB = A5;

// Pot values
int potLastVal1 = 0;
int potLastVal2 = 0;
int potLastVal3 = 0;

// Rotary encoder values
int encLastValA = 0;
int encLastValB = 0;
bool encTriggerUp = false;    // encoder triggered up
bool encTriggerDown = false;  // encoder triggered up

void setup() {
  // Pins
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);
  pinMode(potPin3, INPUT);
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

  readSensor = correctTaper(analogRead(potPin1));
  if (readSensor != potLastVal1) {
    Joystick.setThrottle(readSensor);
    potLastVal1 = readSensor;
  }
  readSensor = correctTaper(analogRead(potPin2));
  if (readSensor != potLastVal2) {
    Joystick.setRxAxis(readSensor);
    potLastVal2 = readSensor;
  }
  readSensor = correctTaper(analogRead(potPin3));
  if (readSensor != potLastVal3) {
    Joystick.setRyAxis(readSensor);
    potLastVal3 = readSensor;
  }

  // Handle encoder
  int readEncA = digitalRead(encPinA);
  int readEncB = digitalRead(encPinB);

  if (encLastValA && !readEncA) {
    // A dropped
    if (readEncB) {
      // A drop & B high = down
      encDown();
    } else {
      // A drop & B low = up
      encUp();      
    }
  }
  if (!encLastValA && readEncA) {
    // A rose
    if (readEncB) {
      // A rise & B high = up
      encUp();
    } else {
      // A rise & B low = down
      encDown();      
    }
  }
  if (encLastValB && !readEncB) {
    // B dropped
    if (readEncA) {
      // B drop & A high = up
      encUp();
    } else {
      // B drop & A low = down
      encDown();      
    }
  }
  if (!encLastValB && readEncB) {
    // B rose
    if (readEncA) {
      // B rise & A high = down
      encDown();
    } else {
      // B rise & A low = up
      encUp();      
    }
  }
  encLastValA = digitalRead(encPinA);
  encLastValB = digitalRead(encPinB);

  if (encTriggerUp && (millis() - encUpTimer > encTriggerPeriod)) {
    Joystick.setButton(0,0);
    encTriggerUp = false;
  }
  if (encTriggerDown && (millis() - encDownTimer > encTriggerPeriod)) {
    Joystick.setButton(1,0);
    encTriggerDown = false;
  }

  // Debug printing
  if (debug) {
    if (millis() - debugTimer > debugPeriod) {
      char buffer[60];
      sprintf(buffer, "Debug: Pots: %4d  %4d  %4d -- Enc: %d  %d Clicks: %u", 
        potLastVal1, potLastVal2, potLastVal3, 
        encLastValA, encLastValB, debugEncClicks);
      Serial.println(buffer);
      debugTimer = millis();
    }
  }

  delay(5);
}

void encUp() {
  if (!encTriggerUp) {
    Joystick.setButton(0,1);
    encTriggerUp = true;
    encUpTimer = millis();
    debugEncClicks++;
  }
}

void encDown() {
  if (!encTriggerUp) {
    Joystick.setButton(1,1);
    encTriggerDown = true;
    encDownTimer = millis();
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
