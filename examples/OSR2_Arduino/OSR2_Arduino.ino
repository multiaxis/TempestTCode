// OSR-Release v4.3
// by TempestMAx 17-9-26
//
// This code is designed to drive the OSR2 stroker robot, but is also intended to be
// used as a template to be adapted to run other t-code controlled arduino projects
// Have fun, play safe!
// History:
// v4.0 - TCode v0.4 compatible, no buttons, no T-wist3 support, 1-6-2026
// v4.1 - TCode v0.4 library bug fixes and API changes, 23-7-26
// v4.2 - D2 function added 13-9-26
// v4.3 - TValve and Vibe/Lube code rewritten and streamlined 17-9-26



// ----------------------------
//  User Settings
// ----------------------------
// These are the setup parameters for an OSR2 on a Romeo BLE mini v2

// Device IDs, for external reference
#define TCODE_DEVICE_INFO "OSR2-Arduino Release v4.3"  // Device and firmware version

// Pin assignments
// T-wist feedback goes on digital pin 2
#define LeftServo_PIN 8    // Left Servo (change to 7 for Romeo v1.1)
#define RightServo_PIN 3   // Right Servo (change to 4 for Romeo v1.1)
#define PitchServo_PIN 9   // Pitch Servo (change to 8 for Romeo v1.1)
#define TwistServo_PIN 10  // Twist Servo
#define ValveServo_PIN 12  // Valve Servo
#define Vibe0_PIN 5        // Vibration motor 1
#define Vibe1_PIN 6        // Vibration motor 2

// Arm servo zeros
// Change these to adjust servo centre positions
// (1500 = centre, 1 complete step = 160)
#define LeftServo_ZERO 1500   // Right Servo
#define RightServo_ZERO 1500  // Left Servo
#define PitchServo_ZERO 1500  // Pitch Servo
#define TwistServo_ZERO 1500  // Twist Servo
#define ValveServo_ZERO 1500  // Valve Servo

// Other functions
#define REVERSE_TWIST_SERVO false // (true/false) Reverse twist servo direction 
#define VALVE_DEFAULT 5000        // Auto-valve default suction level (low-high, 0-9999) 
#define REVERSE_VALVE_SERVO true  // (true/false) Reverse T-Valve direction
#define VIBE_MIN 63               // PWM vibration motor minimum level (0-255)
#define VIBE_TIMEOUT 2000         // Timeout for vibration channels (milliseconds).
#define VIBE_PULSE 5000           // Startup pulse level 
#define VIBE_P_TIME 500           // Startup pulse duration
#define LUBE_V1 false             // (true/false) Lube pump installed instead of vibration channel 1
#define LUBE_PIN 13               // Lube manual input button pin (Connect pin to +5V for ON)
#define LUBE_SPEED 255            // Lube manual pump speed (0-255)

// Libraries used
#include <Servo.h>     // Standard Arduino servo library
#include <TCode.h>     // Tempest's TCode library

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// TCode handler
TCode tcode(TCODE_DEVICE_INFO);
// Declare device axes
Axis stroke("Stroke");
Axis twist("Twist");
Axis roll("Roll");
Axis pitch("Pitch");
Axis vibration0("Vibe1");
Axis vibration1("Vibe2");
Axis valve("Valve",VALVE_DEFAULT);
Axis suck("Suck",VALVE_DEFAULT);
Axis lube("Lube",0);


// Declare servos
Servo LeftServo;
Servo RightServo;
Servo PitchServo;
Servo TwistServo;
Servo ValveServo;

// Declare operating variables
// Position variables
int xLin;
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

  // Start serial connection and report status
  Serial.begin(115200);
  tcode.stringInput("D0\n");
  while (tcode.available() > 0) { char c = tcode.read(); Serial.write(c); }
  tcode.stringInput("D1\n");
  while (tcode.available() > 0) { char c = tcode.read(); Serial.write(c); }

  // Register device axes
  tcode.addAxis("L0", stroke);
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

  // Attach servos
  LeftServo.attach(LeftServo_PIN);
  RightServo.attach(RightServo_PIN);
  PitchServo.attach(PitchServo_PIN);
  TwistServo.attach(TwistServo_PIN);
  ValveServo.attach(ValveServo_PIN);

  // Set vibration PWM pins
  pinMode(Vibe0_PIN,OUTPUT);
  pinMode(Vibe1_PIN,OUTPUT);
  
  // Signal done
  Serial.println("Ready!");
}


// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously
void loop() {

  // Read serial and send to tcode class
  while (Serial.available() > 0) { tcode.byteInput(Serial.read()); }

  // Read tcode buffer and send to serial
  while (tcode.available() > 0) { Serial.write(tcode.read()); }

  // Collect inputs
  // These functions query the t-code object for the position/level at a specified time
  // Number recieved will be an integer, 0-9999
  xLin = tcode.getPosition("L0");
  xRot = tcode.getPosition("R0");
  yRot = tcode.getPosition("R1");
  zRot = tcode.getPosition("R2");
  vibe0 = tcode.getPosition("V0");
  if (!LUBE_V1) { vibe1 = tcode.getPosition("V1"); }
  valveCmd = tcode.getPosition("A0");
  suckCmd = tcode.getPosition("A1");
  if (LUBE_V1) { lubeCmd = tcode.getPosition("A2"); }

  // Override valve position if A1 "suck" command received more recently
  if (tcode.getLast("A1") >= tcode.getLast("A0")) {
    // Valve is upen on the down stroke, specified position on the up stroke
    if (tcode.getVelocity("L0") < -5) { valveCmd = 0; } else { valveCmd = suckCmd; } 
  } 

  // Mix and send servo channels
  // Linear scale inputs to servo appropriate numbers
  int stroke,roll,pitch,valve,twist;
  stroke = map(xLin,0,9999,-350,350);
  roll   = map(yRot,0,9999,-180,180);
  pitch  = map(zRot,0,9999,-350,350);
  twist  = map(xRot,0,9999,1000,-1000);
  if (REVERSE_TWIST_SERVO) { twist = -twist; }
  valve  = map(valveCmd,0,9999,-500,500);
  valve  = constrain(valve, -500, 500);
  if (REVERSE_VALVE_SERVO) { valve = -valve; }

  // Set servo output values
  // Note: 1000 = -45deg, 2000 = +45deg
  LeftServo.writeMicroseconds(LeftServo_ZERO + stroke + roll);
  RightServo.writeMicroseconds(RightServo_ZERO - stroke + roll);
  PitchServo.writeMicroseconds(PitchServo_ZERO - pitch);
  TwistServo.writeMicroseconds(TwistServo_ZERO + twist);
  ValveServo.writeMicroseconds(ValveServo_ZERO + valve);
  // Done with servo channels

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
    analogWrite(pin,map(level,1,9999,VIBE_MIN,255));
  } else {
    analogWrite(pin,0);
  }
}


