// OSR_ESP32 - Beta1
// by TempestMAx 13-10-26
//
// This code is designed to drive the SR6 and OSR2 stroker robots.
// Have fun, play safe!
// History:
// Beta1 - TCode v0.4 library. Bluetooth capability added. TValve and Vibe/Lube code updated. 13-10-26

// ----------------------------
//   User Settings
// ----------------------------

#define OSR2_MODE false   // <----- (true/false) Switch servo outputs to OSR2 mode
#define BLUETOOTH false   // <----- To enable Bluetooth 

// Type of servo being used (160, 180, 270-degree)
#define ServoMaxRange 180  // Servo full range of movement (degrees)

// Pin assignments
#define LowerLeftServo_PIN 15    // Lower Left Servo (OSR2 Left Servo)
#define UpperLeftServo_PIN 2     // Upper Left Servo
#define LowerRightServo_PIN 13   // Lower Right Servo (OSR2 Right Servo)
#define UpperRightServo_PIN 12   // Upper Right Servo
#define LeftPitchServo_PIN 4     // Left Pitch Servo (OSR2 Pitch Servo)
#define RightPitchServo_PIN 14   // Right Pitch Servo
#define TwistServo_PIN 27        // Twist Servo
#define ValveServo_PIN 25        // Valve Servo
#define TwistFeedback_PIN 26     // Twist Servo Feedback
#define Vibe0_PIN 18             // Vibration motor 1
#define Vibe1_PIN 19             // Vibration motor 2

// Arm servo zeros
// Change these to adjust arm positions
// (1500 = centre, 1 complete step = 160)
#define LowerLeftServo_ZERO 1500         // Lower Left Servo (OSR2 Left Servo)
#define UpperLeftServo_ZERO 1500         // Upper Left Servo 
#define LowerRightServo_ZERO 1500        // Lower Right Servo (OSR2 Right Servo)
#define UpperRightServo_ZERO 1500        // Upper Right Servo 
#define LeftPitchServo_ZERO 1500         // Left Pitch Servo (OSR2 Pitch Servo)
#define RightPitchServo_ZERO 1500        // Right Pitch Servo
#define TwistServo_ZERO 1500
#define ValveServo_ZERO 1500

// Servo operating frequencies
#define PitchServo_Freq 330 // Pitch Servos
#define MainServo_Freq 330  // Main Servos
#define TwistServo_Freq 50  // Twist Servo
#define ValveServo_Freq 50  // Valve Servo
#define VibePWM_Freq 8000   // Vibe motor control PWM frequency

// Other functions
#define REVERSE_TWIST_SERVO false // (true/false) Reverse twist servo direction 
#define VALVE_DEFAULT 5000        // Auto-valve default suction level (low-high, 0-9999) 
#define REVERSE_VALVE_SERVO true  // (true/false) Reverse T-Valve direction
#define VIBE_MIN 63               // PWM vibration motor minimum level (0-255)
#define VIBE_TIMEOUT 2000         // Timeout for vibration channels (milliseconds).
#define VIBE_PULSE 5000           // Startup pulse level 
#define VIBE_P_TIME 500           // Startup pulse duration
#define LUBE_V1 false             // (true/false) Lube pump installed instead of vibration channel 1
#define LUBE_PIN 23               // Lube manual input button pin (Connect pin to +5V for ON)
#define LUBE_SPEED 255            // Lube manual pump speed (0-255)


// ----------------------------
//  Auto Settings
// ----------------------------
// Do not change 

// Device IDs for external reference
#if OSR2_MODE != false
  #define DEVICE_NAME "SR6 Beta1"  // Device and firmware version
#else 
  #define DEVICE_NAME "OSR2 ESP32 Beta1"  // Device and firmware version
#endif

// Servo Pulse intervals
#define MainServo_Int 1000000/MainServo_Freq
#define PitchServo_Int 1000000/PitchServo_Freq
#define TwistServo_Int 1000000/TwistServo_Freq
#define ValveServo_Int 1000000/ValveServo_Freq

// Servo microseconds per radian
#define ms_per_rad 114592/ServoMaxRange  // (μs/rad)

// Libraries used
#include <TCode.h>        // Tempest's TCode Library
#include <TCodeComms.h>   // Tempest's Bluetooth Library

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
TCode tcode(DEVICE_NAME);
// Declare device axes
Axis stroke("Stroke");
Axis surge("Surge");
Axis sway("Sway");
Axis twist("Twist");
Axis roll("Roll");
Axis pitch("Pitch");
Axis vibration0("Vibe 1", 0);
Axis vibration1("Vibe 2", 0);
Axis valve("Valve");
Axis suck("Suck", VALVE_DEFAULT);
Axis lube("Lube", 0);

// Declare operating variables
// Position variables
int xLin,yLin,zLin;
// Rotation variables
int xRot,yRot,zRot;
// Vibration variables
int vibe0,vibe0Last,vibe1,vibe1Last;
unsigned long vibe0Start,vibe1Start;
// Lube variables
int lubeCmd;
// Valve variables
int valveCmd,suckCmd;

// Setup function
// This is run once, when the arduino starts
void setup() {

  // Start Serial connection and report status
  Serial.begin(115200);
  tcode.stringInput("D0\n");
  while (tcode.available() > 0) { char c = tcode.read(); Serial.write(c); }
  tcode.stringInput("D1\n");
  while (tcode.available() > 0) { char c = tcode.read(); Serial.write(c); }

  if (BLUETOOTH) { tcodecomms.setup(DEVICE_NAME); } // Bluetooth communication

  // Set SR6 arms to startup positions
  if (!OSR2_MODE) { tcode.stringInput("R2750"); }

  // Register device axes
  tcode.addAxis("L0", stroke);
  if (!OSR2_MODE) {
    tcode.addAxis("L1", surge);
    tcode.addAxis("L2", sway);
  }
  tcode.addAxis("R0", twist);
  tcode.addAxis("R1", roll);
  tcode.addAxis("R2", pitch);
  tcode.addAxis("V0", vibration0);
  if (!LUBE_V1) { tcode.addAxis("V1", vibration1); }
  tcode.addAxis("A0", valve);
  tcode.addAxis("A1", suck);
  if (LUBE_V1) {
    tcode.addAxis("A2", lube);
    pinMode(LUBE_PIN,INPUT);
  }

  // Setup Servo PWM channels
  // Lower Left Servo
  ledcAttach(LowerLeftServo_PIN,MainServo_Freq,16);
  // Upper Left Servo
  ledcAttach(UpperLeftServo_PIN,MainServo_Freq,16);
  // Lower Right Servo
  ledcAttach(LowerRightServo_PIN,MainServo_Freq,16);
  // Upper Right Servo
  ledcAttach(UpperRightServo_PIN,MainServo_Freq,16);
  // Left Pitch Servo
  ledcAttach(LeftPitchServo_PIN,PitchServo_Freq,16);
  // Right Pitch Servo
  ledcAttach(RightPitchServo_PIN,PitchServo_Freq,16);
  // Twist Servo
  ledcAttach(TwistServo_PIN,TwistServo_Freq,16);
  // Valve Servo
  ledcAttach(ValveServo_PIN,ValveServo_Freq,16);

  // Set vibration PWM pins
  // Vibe0 Pin
  ledcAttach(Vibe0_PIN,VibePWM_Freq,8);
  // Vibe1 Pin
  ledcAttach(Vibe1_PIN,VibePWM_Freq,8);

  // Signal done
  Serial.println("Ready!");

}




// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously

void loop() {

  // Read inputs and send to tcode class
  while (Serial.available() > 0) { tcode.byteInput(Serial.read()); }
  if (BLUETOOTH) { while (tcodecomms.available() > 0) { tcode.byteInput(tcodecomms.read()); } }

  // Read TCode class and send to output
  while (tcode.available() > 0) {
    uint8_t b = tcode.read();
    Serial.write(b);
    if (BLUETOOTH) { tcodecomms.write(b); }
  }

  // Collect inputs
  // These functions query the t-code object for the position/level at a specified time
  // Number recieved will be an integer, 0-9999
  xLin = tcode.getPosition("L0");
  if (!OSR2_MODE) {
    yLin = tcode.getPosition("L1");
    zLin = tcode.getPosition("L2");
  }
  xRot = tcode.getPosition("R0");
  yRot = tcode.getPosition("R1");
  zRot = tcode.getPosition("R2");
  vibe0 = tcode.getPosition("V0");
  if (!LUBE_V1) { vibe1 = tcode.getPosition("V1"); }
  valveCmd = tcode.getPosition("A0");
  suckCmd = tcode.getPosition("A1");
  if (LUBE_V1) { lubeCmd = tcode.getPosition("A2"); }

  // If you want to mix your servos differently, enter your code below:

  // Override valve position if A1 "suck" command received more recently
  if (tcode.getLast("A1") >= tcode.getLast("A0")) {
    // Valve is upen on the down stroke, specified position on the up stroke
    if (tcode.getVelocity("L0") < -5) { valveCmd = 0; } else { valveCmd = suckCmd; } 
  } 

  // OSR2 Kinematics
  if (OSR2_MODE) {
    // Calculate arm angles
    // Linear scale inputs to servo appropriate numbers
    int strokeCmd,rollCmd,pitchCmd;
    strokeCmd = map(xLin,0,9999,-350,350);
    rollCmd   = map(yRot,0,9999,-180,180);
    pitchCmd  = map(zRot,0,9999,-350,350);
    ledcWrite(LowerLeftServo_PIN, map(LowerLeftServo_ZERO + strokeCmd + rollCmd,0,MainServo_Int,0,65535));
    ledcWrite(LowerRightServo_PIN, map(LowerRightServo_ZERO - strokeCmd + rollCmd,0,MainServo_Int,0,65535));
    ledcWrite(LeftPitchServo_PIN, map(LeftPitchServo_ZERO - pitchCmd,0,PitchServo_Int,0,65535));
    // Unused servo pins.
    ledcWrite(UpperLeftServo_PIN, map(UpperLeftServo_ZERO,0,MainServo_Int,0,65535));
    ledcWrite(RightPitchServo_PIN, map(RightPitchServo_ZERO,0,PitchServo_Int,0,65535));
    ledcWrite(UpperRightServo_PIN, map(UpperRightServo_ZERO,0,MainServo_Int,0,65535));
  }
  
  // SR6 Kinematics
  else {
    // Calculate arm angles
    int rollCmd,pitchCmd,fwdCmd,thrustCmd,sideCmd;
    int out1,out2,out3,out4,out5,out6;
    rollCmd = map(yRot,0,9999,-3000,3000);
    pitchCmd = map(zRot,0,9999,-2500,2500);
    fwdCmd = map(yLin,0,9999,-3000,3000);
    thrustCmd = map(xLin,0,9999,-6000,6000);
    sideCmd = map(zLin,0,9999,-3000,3000);
    // Main arms
    out1 = SetMainServo(16248 - fwdCmd, 1500 + thrustCmd + rollCmd); // Lower left servo
    out2 = SetMainServo(16248 - fwdCmd, 1500 - thrustCmd - rollCmd); // Upper left servo
    out5 = SetMainServo(16248 - fwdCmd, 1500 - thrustCmd + rollCmd); // Upper right servo
    out6 = SetMainServo(16248 - fwdCmd, 1500 + thrustCmd - rollCmd); // Lower right servo
    // Pitchers
    out3 = SetPitchServo(16248 - fwdCmd, 4500 - thrustCmd,  sideCmd - 1.5*rollCmd, -pitchCmd);
    out4 = SetPitchServo(16248 - fwdCmd, 4500 - thrustCmd, -sideCmd + 1.5*rollCmd, -pitchCmd);
    // Set Servos
    ledcWrite(LowerLeftServo_PIN, map(LowerLeftServo_ZERO - out1,0,MainServo_Int,0,65535));
    ledcWrite(UpperLeftServo_PIN, map(UpperLeftServo_ZERO + out2,0,MainServo_Int,0,65535));
    ledcWrite(LeftPitchServo_PIN, map(constrain(LeftPitchServo_ZERO - out3,LeftPitchServo_ZERO-600,LeftPitchServo_ZERO+1000),0,PitchServo_Int,0,65535));
    ledcWrite(RightPitchServo_PIN, map(constrain(RightPitchServo_ZERO + out4,RightPitchServo_ZERO-1000,RightPitchServo_ZERO+600),0,PitchServo_Int,0,65535));
    ledcWrite(UpperRightServo_PIN, map(UpperRightServo_ZERO - out5,0,MainServo_Int,0,65535));
    ledcWrite(LowerRightServo_PIN, map(LowerRightServo_ZERO + out6,0,MainServo_Int,0,65535));
  }

  // Twist and valve
  int twistCmd,valveCmd;
  twistCmd  = map(xRot,0,9999,1000,-1000);
  if (REVERSE_TWIST_SERVO) { twistCmd = -twistCmd; }
  valveCmd  = map(valveCmd,0,9999,-500,500);
  valveCmd  = constrain(valveCmd, -500, 500);
  if (REVERSE_VALVE_SERVO) { valveCmd = -valveCmd; }
  // Set Servos
  ledcWrite(TwistServo_PIN, map(TwistServo_ZERO + twistCmd,0,TwistServo_Int,0,65535));
  ledcWrite(ValveServo_PIN, map(ValveServo_ZERO + valveCmd,0,ValveServo_Int,0,65535));

  // Done with servo channels


  // Output vibration channels
    // Set vibe outputs - via helper function defined below
  setVibe(Vibe0_PIN, vibe0, vibe0Last, vibe0Start);
  if (!LUBE_V1) {
    setVibe(Vibe1_PIN, vibe1, vibe1Last, vibe1Start);
  } else {
    if (digitalRead(LUBE_PIN)) {  // <- Lube button override
      analogWrite(Vibe1_PIN,LUBE_SPEED);
    } else {
      setVibe(Vibe1_PIN, lubeCmd, vibe1Last, vibe1Start);
    }
  }

  // Timeout functions
  if (millis() - tcode.getLast("V0") > VIBE_TIMEOUT) { tcode.stringInput("V00I500\n"); }
  if (!LUBE_V1) {
    if (millis() - tcode.getLast("V1") > VIBE_TIMEOUT) { tcode.stringInput("V10I500\n"); }
  } else {
    if (millis() - tcode.getLast("A2") > 500) { tcode.stringInput("A20I100\n"); }
  }

}


// Vibe channel helper function
void setVibe(int pin, int level, int& last, unsigned long& start) {
  // Channel start pulse generator
  // -> Avoids a stalled motor starting on low levels
  if (level != 0) {
    if (last == 0) { start = millis(); }
    if ((millis() - start) < VIBE_P_TIME) {
      if (level < VIBE_PULSE) { level = VIBE_PULSE;}
    }
  }
  last = level;
  // Vibe channel output
  // Jump from 0 to minimum sustainable level
  constrain(level, 0, 9999);
  if (level > 0 && level <= 9999) {
    ledcWrite(pin,map(level,1,9999,VIBE_MIN,255));
  } else {
    ledcWrite(pin,0);
  }
}


// Function to calculate the angle for the main arm servos
// Inputs are target x,y coords of receiver pivot in 1/100 of a mm
int SetMainServo(float x, float y) {
  x /= 100; y /= 100;          // Convert to mm
  float gamma = atan2(x,y);    // Angle of line from servo pivot to receiver pivot
  float csq = sq(x) + sq(y);   // Square of distance between servo pivot and receiver pivot
  float c = sqrt(csq);         // Distance between servo pivot and receiver pivot
  float beta = acos((csq - 28125)/(100*c));  // Angle between c-line and servo arm
  int out = ms_per_rad*(gamma + beta - 3.14159); // Servo signal output, from neutral
  return out;
}


// Function to calculate the angle for the pitcher arm servos
// Inputs are target x,y,z coords of receiver upper pivot in 1/100 of a mm
// Also pitch in 1/100 of a degree
int SetPitchServo(float x, float y, float z, float pitch) {
  pitch *= 0.0001745; // Convert to radians
  x += 5500*sin(0.2618 + pitch);
  y -= 5500*cos(0.2618 + pitch);
  x /= 100; y /= 100; z /= 100;   // Convert to mm
  float bsq = 36250 - sq(75 + z); // Equivalent arm length
  float gamma = atan2(x,y);       // Angle of line from servo pivot to receiver pivot
  float csq = sq(x) + sq(y);      // Square of distance between servo pivot and receiver pivot
  float c = sqrt(csq);            // Distance between servo pivot and receiver pivot
  float beta = acos((csq + 5625 - bsq)/(150*c)); // Angle between c-line and servo arm
  int out = ms_per_rad*(gamma + beta - 3.14159); // Servo signal output, from neutral
  return out;
}
