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
const bool debug = false;   // === IMPORTANT ===
unsigned int debugEncClicks = 0;

// Loop timings
const unsigned long debugPeriod = 200;
const unsigned long potTriggerPeriod = 75; // timer pot reading
const unsigned long encTriggerPeriod = 50; // timer button is "pressed"
unsigned long debugTimer = 0;
unsigned long pot1Timer = 0;
unsigned long pot2Timer = 0;
unsigned long pot3Timer = 0;
unsigned long encUpTimer = 0;
unsigned long encDnTimer = 0;

// Arduino HW pin used
const int potPin1 = A0;  // select the input pin for the potentiometer
const int potPin2 = A1;  // select the input pin for the potentiometer
const int potPin3 = A2;  // select the input pin for the potentiometer
const int encPinUp = 2;
const int encPinDn = 3;

// Joystick button assignment
const int joyEncUp = 0;
const int joyEncDn = 1;

// Pot values
int potLastVal1 = 0;
int potLastVal2 = 0;
int potLastVal3 = 0;

// Rotary encoder values
int encLastValUp = 0;
int encLastValDn = 0;
bool encTriggerUp = false;  // encoder triggered up
bool encTriggerDn = false;  // encoder triggered up

void setup() {
  // Pins
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);
  pinMode(potPin3, INPUT);
  pinMode(encPinUp, INPUT);
  pinMode(encPinDn, INPUT);

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

  // Read and update potentiometers periodically
  readSensor = correctTaper(analogRead(potPin1));
  if (readSensor != potLastVal1 && (millis() - pot1Timer > potTriggerPeriod)) {
    Joystick.setThrottle(readSensor);
    potLastVal1 = readSensor;
  }
  readSensor = correctTaper(analogRead(potPin2));
  if (readSensor != potLastVal2 && (millis() - pot2Timer > potTriggerPeriod)) {
    Joystick.setRxAxis(readSensor);
    potLastVal2 = readSensor;
  }
  readSensor = correctTaper(analogRead(potPin3));
  if (readSensor != potLastVal3 && (millis() - pot3Timer > potTriggerPeriod)) {
    Joystick.setRyAxis(readSensor);
    potLastVal3 = readSensor;
  }

  // Handle encoder
  int readEncUp = digitalRead(encPinUp);
  int readEncDn = digitalRead(encPinDn);

  if (encLastValUp && !readEncUp) {
    // A dropped
    if (readEncDn) {
      // A drop & B high = down
      encDown();
    } else {
      // A drop & B low = up
      encUp();      
    }
  }
  if (!encLastValUp && readEncUp) {
    // A rose
    if (readEncDn) {
      // A rise & B high = up
      encUp();
    } else {
      // A rise & B low = down
      encDown();      
    }
  }
  if (encLastValDn && !readEncDn) {
    // B dropped
    if (readEncUp) {
      // B drop & A high = up
      encUp();
    } else {
      // B drop & A low = down
      encDown();      
    }
  }
  if (!encLastValDn && readEncDn) {
    // B rose
    if (readEncUp) {
      // B rise & A high = down
      encDown();
    } else {
      // B rise & A low = up
      encUp();      
    }
  }
  encLastValUp = digitalRead(encPinUp);
  encLastValDn = digitalRead(encPinDn);

  // Release joystick button after timer has lapsed
  if (encTriggerUp && (millis() - encUpTimer > encTriggerPeriod)) {
    Joystick.setButton(joyEncUp,0);
    encTriggerUp = false;
  }
  if (encTriggerDn && (millis() - encDnTimer > encTriggerPeriod)) {
    Joystick.setButton(joyEncDn,0);
    encTriggerDn = false;
  }

  // Debug printing
  if (debug) {
    if (millis() - debugTimer > debugPeriod) {
      char buffer[60];
      sprintf(buffer, "Debug: Pots: %4d  %4d  %4d -- Enc: %d  %d Clicks: %u", 
        potLastVal1, potLastVal2, potLastVal3, 
        encLastValUp, encLastValDn, debugEncClicks);
      Serial.println(buffer);
      debugTimer = millis();
    }
  }
}

// Press joystick "up" button in response to encoder input
void encUp() {
  if (!encTriggerUp) {
    Joystick.setButton(joyEncUp,1);
    encTriggerUp = true;
    encUpTimer = millis();
    debugEncClicks++;
  }
}

// Press joystick "down" button in response to encoder input
void encDown() {
  if (!encTriggerUp) {
    Joystick.setButton(joyEncDn,1);
    encTriggerDn = true;
    encDnTimer = millis();
    debugEncClicks--;
  }
}

// We have received non-linear slider pots, this corrects the B5 logarithmic
// taper to some approximation of linear.
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
