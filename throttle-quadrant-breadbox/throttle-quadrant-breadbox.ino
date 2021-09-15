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
  8, 0,                     // Button count, Hat Switch count
  false, false, false,      // X, Y, Z axis
  true, true, false,        // Rx, Ry, Rz rotations
  false, true,              // Rudder, throttle
  false, false, false);     // Accelerator, brake, steering

// Debug mode
const bool debug = true;   // === IMPORTANT ===
unsigned int debugEncClicks = 0;

// Loop timings
const unsigned long debugPeriod = 200;
const unsigned long potTriggerPeriod = 200;  // timer pot reading
const unsigned long encTriggerPeriod = 75; // timer button is "pressed"
const unsigned long butTriggerPeriod = 50;  // timer button is "pressed"
unsigned long debugTmr = 0;
unsigned long pot1Tmr = 0;
unsigned long pot2Tmr = 0;
unsigned long pot3Tmr = 0;
unsigned long encUpTmr = 0;
unsigned long encDnTmr = 0;
unsigned long revBounceTmr = 0;
unsigned long gearBounceTmr = 0;
unsigned long flapBounceTmr = 0;
unsigned long flapActiveTmr = 0;

// Arduino HW pin used
const byte potPin1 = A0;  // input pin for the potentiometer
const byte potPin2 = A1;  // input pin for the potentiometer
const byte potPin3 = A2;  // input pin for the potentiometer
const byte encPinUp = 2;  // rotary encoder input 1 for elevator trim
const byte encPinDn = 3;  // rotary encoder input 2 for elevator trim
const byte revPin = 4;    // reverse thrust input
const byte gearPin = 5;   // gear lever input
const byte flapPin1 = 6;  // flaps input 1
const byte flapPin2 = 7;  // flaps input 2

// Joystick button assignment
const byte joyEncUp = 0;
const byte joyEncDn = 1;
const byte joyRev = 2;
const byte joyGear = 3;
const byte joyFlapUp = 4;
const byte joyFlapTo = 5;
const byte joyFlapLg = 6;

// Pot values
int potLastVal1 = 2048;
int potLastVal2 = 2048;
int potLastVal3 = 2048;

// Rotary encoder values
byte encLastValUp = 255;
byte encLastValDn = 255;
bool encTriggerUp = false;  // encoder triggered up
bool encTriggerDn = false;  // encoder triggered up

// Button values
bool revTrigger = false;
bool gearTrigger = false;
bool flapTrigger = false;
byte revLastVal = 255;
byte gearLastVal = 255;
byte flapLastVal = 255;

void setup() {
  // Pins
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);
  pinMode(potPin3, INPUT);
  pinMode(encPinUp, INPUT_PULLUP);
  pinMode(encPinDn, INPUT_PULLUP);
  pinMode(revPin, INPUT_PULLUP);
  pinMode(gearPin, INPUT_PULLUP);
  pinMode(flapPin1, INPUT_PULLUP);
  pinMode(flapPin2, INPUT_PULLUP);

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
  int potReadVal = 0;

  // Read and update potentiometers periodically
  potReadVal = correctTaper(analogRead(potPin1));
  if (potReadVal != potLastVal1 && (millis() - pot1Tmr > potTriggerPeriod)) {
    Joystick.setThrottle(potReadVal);
    potLastVal1 = potReadVal;
  }
  potReadVal = correctTaper(analogRead(potPin2));
  if (potReadVal != potLastVal2 && (millis() - pot2Tmr > potTriggerPeriod)) {
    Joystick.setRxAxis(potReadVal);
    potLastVal2 = potReadVal;
  }
  potReadVal = correctTaper(analogRead(potPin3));
  if (potReadVal != potLastVal3 && (millis() - pot3Tmr > potTriggerPeriod)) {
    Joystick.setRyAxis(potReadVal);
    potLastVal3 = potReadVal;
  }

  // Read encoder
  byte encUpReadVal = digitalRead(encPinUp);
  byte encDnReadVal = digitalRead(encPinDn);

  // Handle encoder input
  if (encLastValUp && !encUpReadVal) {
    // A dropped
    if (encDnReadVal) {
      // A drop & B high = down
      encDown();
    } else {
      // A drop & B low = up
      encUp();      
    }
  }
  if (!encLastValUp && encUpReadVal) {
    // A rose
    if (encDnReadVal) {
      // A rise & B high = up
      encUp();
    } else {
      // A rise & B low = down
      encDown();      
    }
  }
  if (encLastValDn && !encDnReadVal) {
    // B dropped
    if (encUpReadVal) {
      // B drop & A high = up
      encUp();
    } else {
      // B drop & A low = down
      encDown();      
    }
  }
  if (!encLastValDn && encDnReadVal) {
    // B rose
    if (encUpReadVal) {
      // B rise & A high = down
      encDown();
    } else {
      // B rise & A low = up
      encUp();      
    }
  }
  encLastValUp = digitalRead(encPinUp);
  encLastValDn = digitalRead(encPinDn);

  // Release encoder triggered joystick button after timer has lapsed
  if (encTriggerUp && (millis() - encUpTmr > encTriggerPeriod)) {
    Joystick.setButton(joyEncUp,0);
    encTriggerUp = false;
  }
  if (encTriggerDn && (millis() - encDnTmr > encTriggerPeriod)) {
    Joystick.setButton(joyEncDn,0);
    encTriggerDn = false;
  }

  // Handle reverse thrust button and debounce
  byte revReadVal = digitalRead(revPin);
  if (!revTrigger && revReadVal != revLastVal) {
    revBounceTmr = millis();
    revTrigger = true;
  }
  if (revTrigger && millis() - revBounceTmr > butTriggerPeriod) {
    // Debounce delay expired, button change was real
    if (revReadVal != revLastVal) {
      // Save new state
      revLastVal = revReadVal;
      // Joystick button inverted to pin change
      if (revReadVal) { 
        Joystick.setButton(joyRev,0);
      } else {
        Joystick.setButton(joyRev,1); 
      }
    }
    revTrigger = false;
  }

  // Handle gear switch with debounce
  byte gearReadVal = digitalRead(gearPin);
  if (!gearTrigger && gearReadVal != gearLastVal) {
    gearBounceTmr = millis();
    gearTrigger = true;
  }
  if (gearTrigger && millis() - gearBounceTmr > butTriggerPeriod) {
    // Debounce delay expired, switch state changed
    if (gearReadVal != gearLastVal) {
      // Save new state
      gearLastVal = gearReadVal;
      // Joystick button follows pin change
      // if (gearReadVal) { 
      //   Joystick.setButton(joyGear,1);
      // } else {
      //   Joystick.setButton(joyGear,0); 
      // }
      Joystick.setButton(joyGear,gearLastVal);
    }
    gearTrigger = false;      
  }

  // Handle flap switch with debounce
  byte flapReadVal = !digitalRead(flapPin1) + (!digitalRead(flapPin2) << 1);
  if (!flapTrigger && flapReadVal != flapLastVal) {
    flapBounceTmr = millis();
    flapTrigger = true;
  }
  if (flapTrigger && millis() - flapBounceTmr > butTriggerPeriod) {
    // Debounce delay expired, switch state changed
    if (flapReadVal != flapLastVal) {
      // Save new state
      flapLastVal = flapReadVal;
      // Trigger the appropriate button to be pushed
      switch (flapReadVal) {
        case 2:
          // Clear any old states
          Joystick.setButton(joyFlapTo,0);
          Joystick.setButton(joyFlapLg,0);
          // Set flaps UP
          Joystick.setButton(joyFlapUp,1);
          break;
        case 0:
          // Clear any old states
          Joystick.setButton(joyFlapUp,0);
          Joystick.setButton(joyFlapLg,0);
          // Set flaps T/O
          Joystick.setButton(joyFlapTo,1);
          break;
        case 1:
          // Clear any old states
          Joystick.setButton(joyFlapUp,0);
          Joystick.setButton(joyFlapTo,0);
          // Set flaps LDG
          Joystick.setButton(joyFlapLg,1);
         break;
        default:
          // Should never happen
          // Clear any old states
          Joystick.setButton(joyFlapUp,0);
          Joystick.setButton(joyFlapTo,0);
          Joystick.setButton(joyFlapLg,0);
          break;
      }
    }
    flapTrigger = false;
  }


  // Debug printing
  if (debug) {
    if (millis() - debugTmr > debugPeriod) {
      char buffer[80];
      sprintf(buffer, "Debug: Pots: %4d  %4d  %4d -- Enc: %d  %d Clicks: %u -- Flaps: %u", 
        potLastVal1, potLastVal2, potLastVal3, 
        encLastValUp, encLastValDn, debugEncClicks, flapReadVal);
      Serial.println(buffer);
      debugTmr = millis();
    }
  }
}

// Press joystick "up" button in response to encoder input
void encUp() {
  if (!encTriggerUp) {
    Joystick.setButton(joyEncUp,1);
    encTriggerUp = true;
    encUpTmr = millis();
    debugEncClicks++;
  }
}

// Press joystick "down" button in response to encoder input
void encDown() {
  if (!encTriggerUp) {
    Joystick.setButton(joyEncDn,1);
    encTriggerDn = true;
    encDnTmr = millis();
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
