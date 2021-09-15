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
const bool debug = false;   // === IMPORTANT ===
byte debugEncDirection = 0;

// Loop timings
//
// Note on the Joystick buttons, the encoder rotation triggers a Joystick
// button press. On my Win10 machine 50ms was the lowest value that
// appeared to be reliably detect the button inupt signal.
const unsigned long debugPeriod = 200;
const unsigned long potBouncePeriod = 200;  // debounce potentiometer read
const unsigned long encBouncePeriod = 10;   // debounce encoder read
const unsigned long encButtonPeriod = 50;   // hold time for joystick button press
const unsigned long butBouncePeriod = 50;   // debounce button read
unsigned long debugTmr = 0;
unsigned long pot1Tmr = 0;
unsigned long pot2Tmr = 0;
unsigned long pot3Tmr = 0;
unsigned long encBounceTmr = 0;
unsigned long encButtonTmr = 0;
unsigned long revBounceTmr = 0;
unsigned long gearBounceTmr = 0;
unsigned long flapBounceTmr = 0;

// Arduino HW pin used
const byte potPin1 = A0;  // input pin for the potentiometer
const byte potPin2 = A1;  // input pin for the potentiometer
const byte potPin3 = A2;  // input pin for the potentiometer
const byte encPin1 = 9;   // rotary encoder input 1 for elevator trim
const byte encPin2 = 10;  // rotary encoder input 2 for elevator trim
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
bool encArmFlag = false;  // tracks active encoder inout change
byte encButtonFlag = 0;   // tracks active Joystick button send
byte encLastVal = 255;

// Button values
bool revArmFlag = false;
bool gearArmFlag = false;
bool flapArmFlag = false;
byte revLastVal = 255;
byte gearLastVal = 255;
byte flapLastVal = 255;

void setup() {
  // Pins
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);
  pinMode(potPin3, INPUT);
  pinMode(encPin1, INPUT_PULLUP);
  pinMode(encPin2, INPUT_PULLUP);
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

  // Initialize serial debugging
  if (debug) { Serial.begin(9600); }
}

void loop() {
  // Handle pots
  int potReadVal = 0;

  // Read and update potentiometers periodically
  potReadVal = correctTaper(analogRead(potPin1));
  if (potReadVal != potLastVal1 && (millis() - pot1Tmr > potBouncePeriod)) {
    Joystick.setThrottle(potReadVal);
    potLastVal1 = potReadVal;
  }
  potReadVal = correctTaper(analogRead(potPin2));
  if (potReadVal != potLastVal2 && (millis() - pot2Tmr > potBouncePeriod)) {
    Joystick.setRxAxis(potReadVal);
    potLastVal2 = potReadVal;
  }
  potReadVal = correctTaper(analogRead(potPin3));
  if (potReadVal != potLastVal3 && (millis() - pot3Tmr > potBouncePeriod)) {
    Joystick.setRyAxis(potReadVal);
    potLastVal3 = potReadVal;
  }

  // Handle elevator trim rotary encoder
  //
  // The two bit gray code from the encoder looks as follows:
  //       ,- pin1
  //      / ,- pin 2
  //     / /
  //    0 0
  //    0 1
  //    1 1
  //    1 0
  //    0 0
  //    0 1
  //
  // I had two thoughs of processing this:
  //     - Convert value to binary (decimal) to detect counting up 
  //       and counting down
  //     - Prepend the previous new read value to the last read value
  //       and detect specific states
  //
  // Because we are just dealing with a two bit gray code and the
  // number of possible values are limited to 4 unique states, I
  // choose the prepend and match state method.
  //
  // For reference purposes, this converts the two bit gray code to
  // binary:
  //
  // byte encReadVal = (digitalRead(encPin1)  << 1)| (digitalRead(encPin1) ^ digitalRead(encPin2));
  //
  // Start by reading the encoder pin states into a two bit value
  // Note: Is there a better way to read both bits at once?
  byte encReadVal = (digitalRead(encPin1)  << 1) | digitalRead(encPin2);
  // Arm debounce timer on pin change detected AND inactive button send
  if (!encArmFlag && encReadVal != encLastVal && !encButtonFlag) {
    encBounceTmr = millis();
    encArmFlag = true;
  }
  // Handle debounce time expiry
  if (encArmFlag && millis() - encBounceTmr > encBouncePeriod) {
    // Debounce delay expired, check on state change still true
    if (encReadVal != encLastVal) {
      // Detect rotary movement based on gray coding pin read
      //
      //   Direction One (call it "up")
      //        ,- new pin 1
      //       / ,- new pin 2
      //      / / ,- last pin 1
      //     / / / ,- last pin 2
      //    / / / /
      //   0 1 0 0
      //   1 1 0 1
      //   1 0 1 1
      //   0 0 1 0
      //   0 1 0 0 
      //   1 1 0 1
      //
      //   Direction One (call it "down")
      //        ,- new pin 1
      //       / ,- new pin 2
      //      / / ,- last pin 1
      //     / / / ,- last pin 2
      //    / / / /
      //   1 0 0 0
      //   0 0 0 1
      //   0 1 1 1
      //   1 1 1 0
      //   1 0 0 0 
      //   0 0 0 1
      //
      byte encDirection = (encReadVal << 2 ) | encLastVal;
      if (debug) { debugEncDirection = encDirection; }
      if (encDirection == 0b0100 ||
          encDirection == 0b1101 ||
          encDirection == 0b1011 ||
          encDirection == 0b0010 ) {
        // Going up, activate Joystick button
        Joystick.setButton(joyEncUp,1);
        encButtonFlag = 1;                 // 1 = the up button
        encButtonTmr = millis();
      } else if (encDirection == 0b1000 ||
          encDirection == 0b0001 ||
          encDirection == 0b0111 ||
          encDirection == 0b1110 ) {
        // Going down, activate Joystick button
        Joystick.setButton(joyEncDn,1);
        encButtonFlag = 2;                 // 2 = the down button
        encButtonTmr = millis();
      } else {
        // Oops an error occured!
      }
      // Save new state and disarm
      encLastVal = encReadVal;
    }
    // Disarm debounce
    encArmFlag = false;
  }
  // Handle encoder triggered Joystick button release
  if (encButtonFlag && millis() - encButtonTmr > encButtonPeriod) {
    if (encButtonFlag == 1) {
      // Clear the up button
      Joystick.setButton(joyEncUp,0);
    } else if (encButtonFlag == 2) {
      // Clear the down button
      Joystick.setButton(joyEncDn,0);
    } else {
    // Oops and error occured!
    }
    // Clear flag as no buttons are pressed any longer
    encButtonFlag = 0;
  }

  // Handle reverse thrust button and debounce
  byte revReadVal = digitalRead(revPin);
  if (!revArmFlag && revReadVal != revLastVal) {
    revBounceTmr = millis();
    revArmFlag = true;
  }
  if (revArmFlag && millis() - revBounceTmr > butBouncePeriod) {
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
    revArmFlag = false;
  }

  // Handle gear switch with debounce
  byte gearReadVal = digitalRead(gearPin);
  if (!gearArmFlag && gearReadVal != gearLastVal) {
    gearBounceTmr = millis();
    gearArmFlag = true;
  }
  if (gearArmFlag && millis() - gearBounceTmr > butBouncePeriod) {
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
    gearArmFlag = false;      
  }

  // Handle flap switch with debounce
  byte flapReadVal = !digitalRead(flapPin1) + (!digitalRead(flapPin2) << 1);
  if (!flapArmFlag && flapReadVal != flapLastVal) {
    flapBounceTmr = millis();
    flapArmFlag = true;
  }
  if (flapArmFlag && millis() - flapBounceTmr > butBouncePeriod) {
    // Debounce delay expired, switch state changed
    if (flapReadVal != flapLastVal) {
      // Save new state
      flapLastVal = flapReadVal;
      // Trigger the appropriate button to be pushed
      switch (flapReadVal) {
        case 0b10:
          // Clear any old states
          Joystick.setButton(joyFlapTo,0);
          Joystick.setButton(joyFlapLg,0);
          // Set flaps UP
          Joystick.setButton(joyFlapUp,1);
          break;
        case 0b00:
          // Clear any old states
          Joystick.setButton(joyFlapUp,0);
          Joystick.setButton(joyFlapLg,0);
          // Set flaps T/O
          Joystick.setButton(joyFlapTo,1);
          break;
        case 0b01:
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
    flapArmFlag = false;
  }


  // Debug printing
  if (debug) {
    if (millis() - debugTmr > debugPeriod) {
      char buffer[80];
      sprintf(buffer, "Debug: Pots: %4d  %4d  %4d -- Enc: %u -- Flaps: %u", 
        potLastVal1, potLastVal2, potLastVal3, 
        debugEncDirection, flapReadVal);
      Serial.println(buffer);
      debugTmr = millis();
    }
  }
}

// // Press joystick "up" button in response to encoder input
// void encUp() {
//   if (!encArmFlag) {
//     Joystick.setButton(joyEncUp,1);
//     encArmFlag = true;
//     encUpTmr = millis();
//     debugEncClicks++;
//   }
// }
// 
// // Press joystick "down" button in response to encoder input
// void encDown() {
//   if (!encArmFlag) {
//     Joystick.setButton(joyEncDn,1);
//     encArmFlagDn = true;
//     encDnTmr = millis();
//     debugEncClicks--;
//   }
// }

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
