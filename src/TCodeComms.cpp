// Only for ESP32s
#if defined(ARDUINO_ARCH_ESP32)

#include "TCodeComms.h"

// Global instance
TCodeComms tcodecomms;

// ---------------------------------------------------------------------------
// Nested callback classes
// ---------------------------------------------------------------------------

class TCodeComms::ServerCallbacks : public BLEServerCallbacks {
  TCodeComms *parent;
public:
  ServerCallbacks(TCodeComms *p) : parent(p) {}
  void onConnect(BLEServer *pServer) override {
    parent->deviceConnected = true;
    Serial.println("BLE device connected");
  }
  void onDisconnect(BLEServer *pServer) override {
    parent->deviceConnected = false;
    Serial.println("BLE device disconnected");
    BLEDevice::startAdvertising();
  }
};

class TCodeComms::RxCallbacks : public BLECharacteristicCallbacks {
  TCodeComms *parent;
public:
  RxCallbacks(TCodeComms *p) : parent(p) {}
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // Use the raw value + length (avoids String / c_str() issues)
    const uint8_t *data = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    if (data && len > 0) {
      parent->addToInputBuffer(data, len);
    }
  }
};

// ---------------------------------------------------------------------------
// Comms implementation
// ---------------------------------------------------------------------------

TCodeComms::TCodeComms()
  : outputIndex(0)
  , pServer(nullptr)
  , pTxCharacteristic(nullptr)
  , deviceConnected(false)
{}

void TCodeComms::setup(String deviceName) {
  // Create the inbound queue before BLE starts (BLE task will write to it)
  inputQueue = xQueueCreate(INPUT_QUEUE_SIZE, sizeof(uint8_t));
  if (!inputQueue) {
    Serial.println("Failed to create BLE input queue");
  }

  BLEDevice::init(deviceName);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks(this));

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Single characteristic: READ + WRITE_NR (matches the web UI)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR
                      );
  pTxCharacteristic->setValue("");
  pTxCharacteristic->setCallbacks(new RxCallbacks(this));

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising started as: " + deviceName);
  Serial.println("Service UUID: " + String(SERVICE_UUID));
}

int TCodeComms::available() {
  if (!inputQueue) return 0;
  return uxQueueMessagesWaiting(inputQueue);
}

uint8_t TCodeComms::read() {
  if (!inputQueue) return 0;
  uint8_t byte = 0;
  if (xQueueReceive(inputQueue, &byte, 0) == pdTRUE) {
    return byte;
  }
  return 0;
}

void TCodeComms::write(uint8_t byte) {
  if (outputIndex < OUTPUT_BUFFER_SIZE - 1) {
    outputBuffer[outputIndex++] = byte;
  } else {
    flushOutput();
    if (outputIndex < OUTPUT_BUFFER_SIZE - 1) {
      outputBuffer[outputIndex++] = byte;
    }
  }

  if (byte == '\n' || byte == '\r') {
    flushOutput();
  }
}

void TCodeComms::flush() {
  flushOutput();
}

bool TCodeComms::isConnected() const {
  return deviceConnected;
}

void TCodeComms::addToInputBuffer(const uint8_t *data, size_t len) {
  if (!inputQueue || !data) return;

  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    // Skip embedded nulls (shouldn't appear in TCode but protect the parser)
    if (b == 0) continue;

    // Non-blocking send; drop the byte only if the queue is completely full
    xQueueSend(inputQueue, &b, 0);
  }
}

void TCodeComms::flushOutput() {
  if (outputIndex == 0) return;
  if (!deviceConnected || pTxCharacteristic == nullptr) {
    outputIndex = 0;
    return;
  }

  // Update the characteristic value so a subsequent read returns the latest line.
  int offset = 0;
  while (offset < outputIndex) {
    int chunk = min(BLE_CHUNK_SIZE, outputIndex - offset);
    pTxCharacteristic->setValue(outputBuffer + offset, chunk);
    offset += chunk;
  }
  outputIndex = 0;
}

#endif // Only for ESP32s