// TCODE AXIS LIBRARY
// by TempestMAx 13-3-26
// This class handles the movement of a single TCode axis
// v0.4.1 Experimental build, 1-6-26
// v0.4.2 setDecelStop() function modified to rebound, now time specified,
//        setAxis() function modified - short INTERVAL commands no longer treated
//        as SHORT commands
// v0.4.3 No Axis changes 13-9-26

#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>   // provides uint32_t, uint16_t, etc.

#define DECEL_CYCLE_TIME 1000
#define LIVE_TRIGGER 25
#define SHORT_MOVE_INTERVAL 50
#define MEAN_INTERVAL_STEPS 5
#define LIVE_CONVERGE_STEPS 10

enum class InputType {
    SHORT,      // no timing parameter — use sensible default interval/speed
    SPEED,      // extended parameter = average speed (pos units per 100 ms)
    INTERVAL    // extended parameter = duration in microseconds
};

enum class MoveStyle {
    RAMPED,     // linear / constant velocity (no easing)
    EASE_IN,    // quadratic start from rest
    EASE_OUT,   // quadratic soft landing
    EASE_BOTH,  // full cubic smoothstep (rest-to-rest)
    GRADIENT    // full Hermite control using supplied start/end gradients
};


// ============================================
//           AXIS CLASS
// ============================================
class Axis {
public:

  // Constructor
  Axis(const char* nameIn, uint16_t startPosIn = 5000);  

  // Returns the name of the axis
  const char* getName() { return name; }

  // Returns the current axis position
  uint16_t getPosition();

  // Returns the current axis velocity
  int32_t getVelocity(int perInterval = 100);

  // Returns the time of the last received command
  unsigned long getLast();

  // High-level axis input handler, used to buffer axis inputs before movement initiated
  void prepAxis(uint16_t targetPos,
              InputType inputType = InputType::SHORT,
              uint32_t extendedParam = 0,
              MoveStyle style = MoveStyle::RAMPED,
              int32_t gradient = 0);

  // High level axis trigger, used to load and execute buffered axis parameters
  bool setAxis();   // Returns true if new parameters are loaded and executed

  // Immediately set axis to this position
  void setPos(uint16_t pos);

  // Immediately halts axis movement
  void stop();

private:
  // Axis name
  const char* name;

  // Buffered parameters - loaded here before being executed
  uint16_t bufferTargetPos;     // Commanded position to move to
  InputType bufferInputType;    // Commanded movement type: SHORT, SPEED, or INTERVAL
  int32_t bufferExtendedParam;  // Associated speed or time interval parameter
  MoveStyle bufferMoveStyle;    // Type of movement - default is RAMPED
  int32_t bufferGradient;       // Associated gradient parameter where approriate
  bool bufferSet;               // Flag to register if the buffer is loaded and ready

  // Current movement parameters
  unsigned long startTime;      // start time for the axis movement
  uint32_t duration;            // axis movement duration in ms
  uint16_t startPos;            // start position of the axis movement (0-10000)
  uint16_t endPos;              // end position of the axis movement (0-10000)
  bool gradMode;                // is the axis operating in gradient mode?

  // Live motion parameters
  unsigned long lastShortTime;
  uint16_t lastShortPos;
  uint32_t meanLiveInterval;
  uint16_t liveCovergeSteps;
  bool liveMode;                // is the axis operating in live mode?

  // All coefficients stored in Q16.16 fixed-point (16 integer bits, 16 fractional)
  int64_t coeffA;        // τ³ coefficient
  int64_t coeffB;        // τ² coefficient
  int64_t coeffC;        // τ   coefficient
  int64_t coeffD;        // constant term

  // Short command handler function
  // Returns true if the current movement command should be skipped
  bool shortCmd(uint32_t& durationIn);       // Movement duration

  // Sets up a cubic Hermite spline movement                                 
  void setCubic(uint32_t durationIn,         // Movement duration
                uint16_t startPosIn,         // Movement starting position (0-10000)
                uint16_t endPosIn,           // Movement ending position (0-10000)
                int32_t startGradient = 0,   // Velocity at start, in position units per 100 microseconds
                int32_t endGradient = 0);    // Velocity at end, in position units per 100 microseconds
  
  // Sets up a linear or eased in/out movement
  void setMotion(uint32_t durationIn,        // movement duration
                uint16_t startPosIn,         // Movement starting position (0-10000)
                uint16_t endPosIn,           // Movement ending position (0-10000)
                bool easeIn = false,         // Is there a "<" ease in parameter on the movement?
                bool easeOut = false);       // Is there a ">" ease out parameter on the movement?

  // Sets up a smooth deceleration movement.
  void setDecelStop(uint16_t currentPos,     // Position at the instant braking begins
                int32_t  currentVel,         // Velocity at the instant braking begins (from getVelocityAtTime(duration))
                uint32_t maxDuration);       // max deceleration deceleration cycle time.

  // Sets up a follow-up movement if a current movement times out.
  void setTimeout();

  // Returns the interpolated position at the given elapsed time.
  // Uses pure integer fixed-point arithmetic for speed on AVR.
  uint16_t getPositionFromCurve(unsigned long elapsedUs);

  // Returns instantaneous velocity at the given elapsed time.
  // Output unit matches Speed and Gradient command inputs: position units per 100 microseconds.
  int32_t getVelocityFromCurve(unsigned long elapsedUs, int perInterval = 100);

};

#endif