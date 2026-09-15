// TCODE LIBRARY
// by TempestMAx 13-9-26
// This class acts as a handler for all TCode axes
// v0.4.1 Experimental build, 1-6-26
// v0.4.2 Added getPosition(), getVelocity(), getlast() axis access functions
// v0.4.3 - D2 function added 13-9-26


#ifndef TCODE_H
#define TCODE_H


#if defined(ARDUINO_ARCH_ESP32)
    #include <Preferences.h>
#else
    #include <EEPROM.h>
#endif

#include "Axis.h"      // 
#include <Arduino.h>   // provides uint32_t, uint16_t, etc.

#ifndef TCODE_VERSION_INFO
#define TCODE_VERSION_INFO  "TCode v0.4"
#endif


// ============================================
//           TCODE CLASS
// ============================================
class TCode {
public:
    TCode(const char* nameIn);

    // Register an axis to its official TCode channel.
    // Pass the channel exactly as the protocol expects ("L0", "R3", "V7", "A1", etc.).
    // Returns false if the channel is invalid or already taken.
    bool addAxis(const char* channel, Axis& axis);

    // Typed accessors
    Axis* Lin(uint8_t index) const;  // L0..L9
    Axis* Rot(uint8_t index) const;  // R0..R9
    Axis* Vib(uint8_t index) const;  // V0..V9
    Axis* Aux(uint8_t index) const;  // A0..A9

    // General lookup
    Axis* getAxis(const char* channel) const;

    // Get current position of any axis
    uint16_t getPosition(char letter, uint8_t index) const;
    uint16_t getPosition(const char* channel) const;

    // Get current velocity of any axis
    int32_t getVelocity(char letter, uint8_t index, int perInterval = 100) const;
    int32_t getVelocity(const char* channel, int perInterval = 100) const;

    // Get current velocity of any axis
    unsigned long getLast(char letter, uint8_t index) const;
    unsigned long getLast(const char* channel) const;

    // Utility
    uint8_t getRegisteredCount() const { return _count; }

    // This feeds in the inputs that command the axes
    void byteInput(uint8_t byte);

    // This can be used to feed a string into the interpreter
    void stringInput(const char* str);

    // Output buffer — read exactly like Serial
    size_t available();
    int read();

    // All stop command, used by DSTOP
    void stopAll();

private:
    // ESP32 data storage
    #if defined(ARDUINO_ARCH_ESP32)
        Preferences _prefs;
    #endif

    // Device name
    const char* _name;

    // Internal storage — fixed arrays of pointers, one per channel type
    Axis* _Lin[10] = {nullptr};
    Axis* _Rot[10] = {nullptr};
    Axis* _Vib[10] = {nullptr};
    Axis* _Aux[10] = {nullptr};
    uint8_t _count = 0;

    // Parser buffer
    char _cmdBuffer[48] = {0};
    uint8_t _cmdLen = 0;

    // Simple output buffer for D responses
    char _outputBuffer[64] = {0};
    uint8_t _outputLen = 0;
    uint8_t _outputPos = 0;
    int8_t _D2dumpSlot = -1; 

    // Command processing
    void processCommand(const char* cmd);
    void processAxisCommand(const char* cmd);
    void processDeviceCommand(const char* cmd);
    void processSaveCommand(const char* cmd);
    void executeAll();

    // Output functions
    void queueResponse(const char* text);
    void queueD2AxisReport(char letter, uint8_t index);
    void queueNextD2DumpLine();

    // Helper function that turns "L3" into letter='L' and index=3
    bool parseChannel(const char* channel, char& letter, uint8_t& index) const;
};

#endif