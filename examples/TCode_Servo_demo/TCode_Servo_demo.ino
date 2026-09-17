// TCode Servo Example Sketch
// by TempestMAx 16-9-26
// This sketch controls a single servo using TCode (L0 axis)
// Use this sketch with a standard servo

// Device IDs, for external reference
#define TCODE_DEVICE_INFO "TCode Servo"

// Define pins
#define Servo_PIN 3        // Connect servo signal cable to this pin
// (Don't forget to also connect the servo ground to Arduino ground!)
#define PWM_Range 1000     // 2000 => 180 degrees, 1000 => 90 degrees

#include <TCode.h>         // Tempest's TCode library
#include <Servo.h>         // Arduino Servo library

// ----------------------------
//   SETUP
// ----------------------------

// Declare classes
// TCode handler
TCode tcode(TCODE_DEVICE_INFO);
// Declare device axes
Axis stroke("Stroke");
// Declare servo
Servo Main;

void setup() {

  // Start serial connection and report status
  Serial.begin(115200);
  tcode.stringInput("D0\n");
  while (tcode.available() > 0) { Serial.write(tcode.read()); }
  tcode.stringInput("D1\n");
  while (tcode.available() > 0) { Serial.write(tcode.read()); }

  // Register device axes
  tcode.addAxis("L0", stroke);

  // Register servo
  Main.attach(Servo_PIN);

  // Signal done
  Serial.println("Ready!");

}

// ----------------------------
//   MAIN
// ----------------------------
void loop() {

  // Read serial and send to tcode class
  while (Serial.available() > 0) {
    tcode.byteInput(Serial.read());  // Send the serial bytes to the t-code object
  }

  // Read tcode buffer and send to serial
  while (tcode.available() > 0) {
    Serial.write(tcode.read());      // Send the tcode output bytes to serial
  }

  // Collect inputs
  int stroke = tcode.getPosition("L0");

  // Set servo position
  int microsec = map(stroke, 0, 9999, 1500 - PWM_Range/2, 1500 + PWM_Range/2);
  Main.writeMicroseconds(microsec);

}