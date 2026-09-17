// TCode Vibrator Example Sketch
// by TempestMAx 16-9-26
// This sketch controls two TCode vibration channels output via two arduino PWM pins
// Use this sketch with power transistors or motor controllers to control vibration motors

// Device IDs, for external reference
#define TCODE_DEVICE_INFO "TCode Vibrator"

// Define pins
#define Vibe0_PIN 5        // Vibration motor 1
#define Vibe1_PIN 6        // Vibration motor 2

// Define parameters
#define Vibe_MIN 63        // PWM vibe minimum level (0-255)
#define Vibe_PULSE 5000    // Startup pulse level 
#define Vibe_P_TIME 500    // Startup pulse duration
#define Vibe_TIMEOUT 3000  // Vibration cutoff time if no signals received

#include <TCode.h>         // Tempest's TCode library

// ----------------------------
//   SETUP
// ----------------------------

// Declare classes
// TCode handler
TCode tcode(TCODE_DEVICE_INFO);
// Declare device axes
Axis vibration0("Vibe0",0);  // <-- Set initial value 0
Axis vibration1("Vibe1",0);  // <-- Set initial value 0

// Declare vibration channel handler variables
int vibe0Last = 0;
unsigned long vibe0Start = 0;
int vibe1Last = 0;
unsigned long vibe1Start = 0; 

void setup() {

  // Start serial connection and report status
  Serial.begin(115200);
  tcode.stringInput("D0\n");
  while (tcode.available() > 0) { Serial.write(tcode.read()); }
  tcode.stringInput("D1\n");
  while (tcode.available() > 0) { Serial.write(tcode.read()); }

  // Register device axes
  tcode.addAxis("V0", vibration0);
  tcode.addAxis("V1", vibration1);

  // Set vibration PWM pins
  pinMode(Vibe0_PIN,OUTPUT);
  pinMode(Vibe1_PIN,OUTPUT);

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
  int vibe0 = tcode.getPosition("V0");
  int vibe1 = tcode.getPosition("V1");

  // Set vibe outputs - via helper function defined below
  setVibe(Vibe0_PIN, vibe0, vibe0Last, vibe0Start);
  setVibe(Vibe1_PIN, vibe1, vibe1Last, vibe1Start);

  // Timeout functions
  if (millis() - tcode.getLast("V0") > Vibe_TIMEOUT) { tcode.stringInput("V00I500\n"); }
  if (millis() - tcode.getLast("V1") > Vibe_TIMEOUT) { tcode.stringInput("V10I500\n"); }

}


// Vibe channel helper function
void setVibe(int pin, int level, int& last, unsigned long& start) {
  // Channel start pulse generator
  // -> Avoids a stalled motor starting on low levels
  if (level != 0) {
    if (last == 0) { start = millis(); }
    if ((millis() - start) < Vibe_P_TIME) {
      if (level < Vibe_PULSE) { level = Vibe_PULSE;}
    }
  }
  last = level;
  // Vibe channel output
  // Jump from 0 to minimum sustainable level
  constrain(level, 0, 9999);
  if (level > 0 && level <= 9999) {
    analogWrite(pin,map(level,1,9999,Vibe_MIN,255));
  } else {
    analogWrite(pin,0);
  }
}
