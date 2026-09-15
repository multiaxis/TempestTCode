#include "Axis.h"       // TCode Axis library
#include "TCode.h"      // TCode library header
#include <ctype.h>      // toupper, isdigit
#include <stdlib.h>     // strtol
#include <string.h>     // strlen
#include <stdio.h>      // snprintf

// ============================================
//           TCODE CLASS
// ============================================
TCode::TCode(const char* nameIn)
    :   _name(nameIn)
{}

// Registers an axis with the TCode class for use
bool TCode::addAxis(const char* channel, Axis& axis) {
    char letter;
    uint8_t index;
    if (!parseChannel(channel, letter, index)) return false;     // invalid channel like "X5"

    Axis** target = nullptr;                                     // pointer-to-pointer: we'll decide which array
    switch (letter) {
        case 'L': target = _Lin; break;
        case 'R': target = _Rot; break;
        case 'V': target = _Vib; break;
        case 'A': target = _Aux; break;
    }
    if (target[index] != nullptr) return false;                  // already occupied

    target[index] = &axis;                                       // store the address of the caller's Axis
    _count++;                                                    // just for your convenience
    return true;
}

// Typed accessors - Axis pointer getter functions
Axis* TCode::Lin(uint8_t index) const { return (index < 10) ? _Lin[index] : nullptr; }
Axis* TCode::Rot(uint8_t index) const { return (index < 10) ? _Rot[index] : nullptr; }
Axis* TCode::Vib(uint8_t index) const { return (index < 10) ? _Vib[index] : nullptr; }
Axis* TCode::Aux(uint8_t index) const { return (index < 10) ? _Aux[index] : nullptr; }

// Axis pointer getter for general lookup
Axis* TCode::getAxis(const char* channel) const {
    char letter;
    uint8_t index;
    if (!parseChannel(channel, letter, index)) return nullptr;
    switch (letter) {
        case 'L': return Lin(index);   // notice we call the new friendly name internally
        case 'R': return Rot(index);
        case 'V': return Vib(index);
        case 'A': return Aux(index);
    }
    return nullptr;
}

// Function(s) to access axis positions via the TCode class
uint16_t TCode::getPosition(char letter, uint8_t index) const {
    Axis* ax = nullptr;
    char l = toupper(letter);
    switch (l) {
        case 'L': ax = Lin(index); break;
        case 'R': ax = Rot(index); break;
        case 'V': ax = Vib(index); break;
        case 'A': ax = Aux(index); break;
    }
    return ax ? ax->getPosition() : 5000;  // 5000 for unregistered axis
}

uint16_t TCode::getPosition(const char* channel) const {
    char letter;
    uint8_t index;
    if (!parseChannel(channel, letter, index)) return 5000;
    return getPosition(letter, index);
}

// Function(s) to access axis velocity via the TCode class
int32_t TCode::getVelocity(char letter, uint8_t index, int perInterval) const {
    Axis* ax = nullptr;
    char l = toupper(letter);
    switch (l) {
        case 'L': ax = Lin(index); break;
        case 'R': ax = Rot(index); break;
        case 'V': ax = Vib(index); break;
        case 'A': ax = Aux(index); break;
    }
    return ax ? ax->getVelocity(perInterval) : 0;  // 0 for unregistered axis
}

int32_t TCode::getVelocity(const char* channel, int perInterval) const {
    char letter;
    uint8_t index;
    if (!parseChannel(channel, letter, index)) return 0;
    return getVelocity(letter, index, perInterval);
}

// Function(s) to access time of last axis command recieved via the TCode class
unsigned long TCode::getLast(char letter, uint8_t index) const {
    Axis* ax = nullptr;
    char l = toupper(letter);
    switch (l) {
        case 'L': ax = Lin(index); break;
        case 'R': ax = Rot(index); break;
        case 'V': ax = Vib(index); break;
        case 'A': ax = Aux(index); break;
    }
    return ax ? ax->getLast() : 0;  // 0 for unregistered axis
}

unsigned long TCode::getLast(const char* channel) const {
    char letter;
    uint8_t index;
    if (!parseChannel(channel, letter, index)) return 0;
    return getLast(letter, index);
}

// Takes byte inputs and processes them into the command buffer
// Looks for space and newline characters to separate and execute commands
void TCode::byteInput(uint8_t byte) {
    if (byte == ' ' || byte == '\n' || byte == '\r') {
        if (_cmdLen > 0) {
            _cmdBuffer[_cmdLen] = '\0';
            processCommand(_cmdBuffer);
            _cmdLen = 0;
        }
        if (byte == '\n' || byte == '\r') {
            executeAll();
        }
    } else if (_cmdLen < 47) {
        _cmdBuffer[_cmdLen++] = (char)byte;
    }
}

// Function to feed in incoming data for interpretation as a string
void TCode::stringInput(const char* str) {
    if (!str) return;
    while (*str) {
        byteInput((uint8_t)*str);
        ++str;
    }
}

// Reads off how many bytes are in the out buffer
size_t TCode::available() {
    size_t count = _outputLen - _outputPos;
    // if buffer empty and D2 output dump available, dump next D2 line
    if (!count && _D2dumpSlot >= 0) {
        queueNextD2DumpLine();
        count = _outputLen - _outputPos;
    }
    return count;
}

// Reads off a byte from the out buffer
int TCode::read() {
    if (_outputPos >= _outputLen) return -1;
    return (unsigned char)_outputBuffer[_outputPos++];
}

// Stop all channels
void TCode::stopAll() {
    for (uint8_t i = 0; i < 10; ++i) {
        if (_Lin[i]) _Lin[i]->stop();
        if (_Rot[i]) _Rot[i]->stop();
        if (_Vib[i]) _Vib[i]->setPos(0);   // Vibration channels go to zero
        if (_Aux[i]) _Aux[i]->stop();
    }
}


// Top-level command dispatcher
void TCode::processCommand(const char* cmd) {
    if (!cmd || strlen(cmd) == 0) return;

    char first = toupper(cmd[0]);

    if (first == 'L' || first == 'R' || first == 'V' || first == 'A') {
        processAxisCommand(cmd);
    } else if (first == 'D') {
        processDeviceCommand(cmd);
    } else if (first == '$') {
        processSaveCommand(cmd);
    }
    // anything else is silently ignored
}

// Axis command parser
void TCode::processAxisCommand(const char* cmd) {
    char letter;
    uint8_t idx;
    if (!parseChannel(cmd, letter, idx)) return; // Check it's a valid axis address type

    // Check it's an axis that's registered
    Axis* ax = nullptr;
    switch (letter) {
        case 'L': ax = Lin(idx); break;
        case 'R': ax = Rot(idx); break;
        case 'V': ax = Vib(idx); break;
        case 'A': ax = Aux(idx); break;
    }
    if (!ax) return;

    // Parse target position (up to first 4 digits after channel)
    uint16_t targetPos = 0;
    size_t i = 2;  // skip letter + digit
    size_t len = strlen(cmd);
    if (len < 3) return;
    if (!isdigit(cmd[i])) return;
    while (i < len && isdigit(cmd[i]) && i < 6) { // digits 2-5 can be axis position
        targetPos = targetPos * 10 + (cmd[i] - '0');
        ++i;
    }
    size_t j = i;
    while (j < 6) { targetPos *= 10; ++j; } // fill in any missing digits 3-5

    // Set default values
    InputType inputType = InputType::SHORT;
    uint32_t extendedParam = 0;
    MoveStyle style = MoveStyle::RAMPED;
    int32_t gradient = 0;
    bool hasEaseIn = false;
    bool hasEaseOut = false;
    bool hasGradient = false;

    // Read the speed or interval parameter
    if (i < len) {
        char c = toupper(cmd[i]);
        if (c == 'S' || c == 'I') {
            char* endptr;
            long val = strtol(cmd + i + 1, &endptr, 10);
            if ((endptr > cmd + i + 1) && val > 0) {
                //if (val > LONG_COMMAND_MAX) val = LONG_COMMAND_MAX; // Limit this in the Axis class
                extendedParam = (uint32_t)val;
                inputType = (c == 'S') ? InputType::SPEED : InputType::INTERVAL;
                i = endptr - cmd;
            }
        }
    }

    // Look for easing parameters
    while (i < len) {
        char c = toupper(cmd[i]);
        if (c == '<') {
            hasEaseIn = true;
        } else if (c == '>') {
            hasEaseOut = true;
        } else if (c == 'G') {  // Shift to gradient interpreter
            break;
        }
        ++i;
    }

    // Read the gradient parameter
    if (i < len) {
        char c = toupper(cmd[i]);
        if (c == 'G') {
            char* endptr;
            long val = strtol(cmd + i + 1, &endptr, 10);
            if (endptr > cmd + i + 1) {
                gradient = (int32_t)val;
                hasGradient = true;
                i = endptr - cmd;
            }
        }
    }

    // Decide final MoveStyle 
    if (inputType == InputType::SHORT) {    // No easing or gradient if it's a short
        style = MoveStyle::RAMPED;
        gradient = 0;
    } else if (hasGradient) {               // G overrides any < >
        style = MoveStyle::GRADIENT;
    } else if (hasEaseIn && hasEaseOut) {
        style = MoveStyle::EASE_BOTH;
    } else if (hasEaseIn) {
        style = MoveStyle::EASE_IN;
    } else if (hasEaseOut) {
        style = MoveStyle::EASE_OUT;
    }

    // Send the interpretted values to the axis
    ax->prepAxis(targetPos, inputType, extendedParam, style, gradient);
}

// "D" or Device command parser
void TCode::processDeviceCommand(const char* cmd) {
    if (strcmp(cmd, "D0") == 0) {
        queueResponse(_name);
    } else if (strcmp(cmd, "D1") == 0) {
        queueResponse(TCODE_VERSION_INFO);
    } else if (strcmp(cmd, "D2") == 0) {
        _D2dumpSlot = 0;
        queueNextD2DumpLine();
    } else if (strcmp(cmd, "DSTOP") == 0) {
        stopAll();
        queueResponse("STOP");
    }
    // anything else is silently ignored
}

// "$" or Save command parser
void TCode::processSaveCommand(const char* cmd) {
    if (!cmd || cmd[0] != '$') return;

    size_t len = strlen(cmd);
    // Minimum viable: $L0-0-1  (8 chars). Full form $L0-0000-9999 is 13.
    if (len < 8) return;

    char letter;
    uint8_t index;
    // cmd+1 is "L0-0000-9999" — parseChannel wants letter + digit
    if (!parseChannel(cmd + 1, letter, index)) return;

    // After "$TX" we require a hyphen
    if (cmd[3] != '-') return;

    char* endMin;
    long ymin = strtol(cmd + 4, &endMin, 10);
    if (endMin == cmd + 4) return;          // no digits after first hyphen
    if (*endMin != '-') return;             // second hyphen required
    if (ymin < 0 || ymin > 9999) return;

    char* endMax;
    long ymax = strtol(endMin + 1, &endMax, 10);
    if (endMax == endMin + 1) return;       // no digits after second hyphen
    if (ymax < 0 || ymax > 9999) return;

    // Optional: reject leftover junk after the max (spaces already stripped by byteInput)
    if (*endMax != '\0') return;

    Axis* ax = nullptr;
    switch (letter) {
        case 'L': ax = Lin(index); break;
        case 'R': ax = Rot(index); break;
        case 'V': ax = Vib(index); break;
        case 'A': ax = Aux(index); break;
    }
    if (!ax) return;                        // valid format, but that channel isn't registered

    #if defined(ARDUINO_ARCH_ESP32)
        // ESP32 preferences parameter save
        _prefs.begin("tcode", false);
        ymin = constrain(ymin,0,9999);
        ymax = constrain(ymax,0,9999);
        char key[4];
        key[0] = (char)tolower(letter);
        key[1] = (char)('0' + index);
        key[3] = '\0';
        key[2] = 'l'; 
        _prefs.putInt(key, (int)ymin);
        key[2] = 'u';
        _prefs.putInt(key, (int)ymax);
        _prefs.end();
    #else
        // Arduino-style EEPROM write, as on previous TCode devices
        int memIndex = 0;
        switch (letter) {
            case 'L': memIndex = 0; break;
            case 'R': memIndex = 80; break;
            case 'V': memIndex = 160; break;
            case 'A': memIndex = 240; break;
        }
        memIndex += 8*index;
        ymin = constrain(ymin,0,9999);
        EEPROM.put(memIndex, ymin-1);
        ymax = constrain(ymax,0,9999);
        EEPROM.put(memIndex+4, ymax-10000);
    #endif

    // Return report if preferences changed
    queueD2AxisReport(letter, index);

}

// Execute queued commands on all axes
void TCode::executeAll() {
    for (uint8_t i = 0; i < 10; ++i) {
        if (_Lin[i]) _Lin[i]->setAxis();
        if (_Rot[i]) _Rot[i]->setAxis();
        if (_Vib[i]) _Vib[i]->setAxis();
        if (_Aux[i]) _Aux[i]->setAxis();
    }
}

// Clears output buffer and loads a string for output
void TCode::queueResponse(const char* text) {
    _outputLen = 0;
    _outputPos = 0;
    if (text) {
        while (*text && _outputLen < 63) {
            _outputBuffer[_outputLen++] = *text++;
        }
    }
    if (_outputLen < 63) { _outputBuffer[_outputLen++] = '\r'; }
    if (_outputLen < 63) { _outputBuffer[_outputLen++] = '\n'; }
}

// Puts a specific D2 output report into the output buffer
void TCode::queueD2AxisReport(char letter, uint8_t index) {
    Axis* ax = nullptr;
    switch (toupper(letter)) {
        case 'L': ax = Lin(index); break;
        case 'R': ax = Rot(index); break;
        case 'V': ax = Vib(index); break;
        case 'A': ax = Aux(index); break;
    }
    if (!ax) return;

    int ymin = 0;
    int ymax = 9999;
    #if defined(ARDUINO_ARCH_ESP32)
        // ESP32 preferences parameter load
        _prefs.begin("tcode", true);

        char key[4];
        key[0] = (char)tolower(letter);
        key[1] = (char)('0' + index);
        key[3] = '\0';
        key[2] = 'u'; 
        ymax = _prefs.getInt(key, 9999);
        key[2] = 'l';
        ymin = _prefs.getInt(key, 0);
        _prefs.end();
    #else
        // Arduino-style EEPROM read, as on previous TCode devices
        int memIndex = 0;
        switch (letter) {
            case 'L': memIndex = 0; break;
            case 'R': memIndex = 80; break;
            case 'V': memIndex = 160; break;
            case 'A': memIndex = 240; break;
        }
        memIndex += 8*index;
        EEPROM.get(memIndex, ymin);
        ymin += 1;
        EEPROM.get(memIndex+4, ymax);
        ymax += 10000;
    #endif
    ymin = constrain(ymin,0,9999);
    ymax = constrain(ymax,0,9999);

    const char* _name = ax->getName();
    if (!_name) _name = "";

    char line[64];
    snprintf(line, sizeof(line), "%c%u %u %u %s",
             (char)toupper(letter),
             (unsigned)index,
             (unsigned)ymin,
             (unsigned)ymax,
             _name);
    queueResponse(line);
}

// Puts the next D2 output report into the output buffer 
void TCode::queueNextD2DumpLine() {
    while (_D2dumpSlot >= 0 && _D2dumpSlot < 40) {
        uint8_t bank  = _D2dumpSlot / 10;   // 0=L 1=R 2=V 3=A
        uint8_t index = _D2dumpSlot % 10;
        _D2dumpSlot++;

        Axis* ax = nullptr;
        char letter = 'L';
        switch (bank) {
            case 0: letter = 'L'; ax = Lin(index); break;
            case 1: letter = 'R'; ax = Rot(index); break;
            case 2: letter = 'V'; ax = Vib(index); break;
            case 3: letter = 'A'; ax = Aux(index); break;
        }
        if (!ax) continue;

        queueD2AxisReport(letter, index);
        return;   // one line only
    }
    _D2dumpSlot = -1;   // finished
}

// Helper function that verifies that a channel named at the start of a string is registered
bool TCode::parseChannel(const char* cmd, char& letter, uint8_t& index) const {
    if (!cmd || strlen(cmd) < 2) return false;
    letter = toupper(cmd[0]);
    if (cmd[1] < '0' || cmd[1] > '9') return false;
    index = cmd[1] - '0';
    return (letter == 'L' || letter == 'R' || letter == 'V' || letter == 'A');
}
