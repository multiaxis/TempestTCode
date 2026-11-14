// Only for ESP32s
#if defined(ARDUINO_ARCH_ESP32)

#ifndef TCODECOMMS_H
#define TCODECOMMS_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Custom TCode BLE service (matches ayvasoftware/osr-esp32 and common web UIs)
#define SERVICE_UUID           "ff1b451d-3070-4276-9c81-5dc5ea1043bc"
#define CHARACTERISTIC_UUID    "c5f1543e-338d-47a0-8525-01e3c621359d"

#define INPUT_QUEUE_SIZE   1024   // bytes of inbound TCode from BLE
#define OUTPUT_BUFFER_SIZE 128
#define BLE_CHUNK_SIZE     20

class TCodeComms {
public:
  TCodeComms();

  // Call this in setup() to initialise Bluetooth LE
  void setup(String deviceName);

  // Returns number of bytes available from Bluetooth input
  int available();

  // Read one byte from Bluetooth input (call only if available() > 0)
  // Feed directly into tcode.byteInput()
  uint8_t read();

  // Write one byte to the output buffer. When a newline is received the buffered
  // line (including the newline) is sent over Bluetooth.
  void write(uint8_t byte);

  // Optional: force send any pending buffered data
  void flush();

  bool isConnected() const;

private:
  // FreeRTOS queue: BLE task -> main loop (thread-safe)
  QueueHandle_t inputQueue = nullptr;

  // Output line buffer (TCode -> BLE)
  uint8_t outputBuffer[OUTPUT_BUFFER_SIZE];
  int outputIndex;

  // BLE objects
  BLEServer *pServer;
  BLECharacteristic *pTxCharacteristic;
  volatile bool deviceConnected;

  void addToInputBuffer(const uint8_t *data, size_t len);
  void flushOutput();

  // Callback classes (defined in .cpp)
  class ServerCallbacks;
  class RxCallbacks;
};

// Global instance
extern TCodeComms tcodecomms;

#endif // COMMS_H

#endif // Only for ESP32s
