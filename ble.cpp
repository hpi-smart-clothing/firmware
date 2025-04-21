#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "config.h"
#include "ble.h"

// IDs to create BLE Services and Characteristics
#define IMU_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define VIBRATION_SERVICE_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_VIBRATION_LEFT "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_VIBRATION_RIGHT "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_IMU "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pVRightCharacteristic;
BLECharacteristic *pVLeftCharacteristic;
BLECharacteristic *pIMUCharacteristic;
BLECallback vibrationCallbackRight = nullptr;
BLECallback vibrationCallbackLeft = nullptr;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    delay(500);
    pServer->startAdvertising();
    Serial.println("Client disconnected \nstart advertising");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    if (vibrationCallbackRight != nullptr)
    {
      vibrationCallbackRight();
    }
  }
};

// Callback for left vibration
class VibrationLeftCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    vibrationCallbackLeft();
  }
};

// Callback for right vibration
class VibrationRightCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    vibrationCallbackRight();
  }
};

void setVibrationLeftCallback(BLECallback callbackFunction)
{
  vibrationCallbackLeft = callbackFunction;
}

void setVibrationRightCallback(BLECallback callbackFunction)
{
  vibrationCallbackRight = callbackFunction;
}

void setupBLE()
{
  // Create the BLE Device
  BLEDevice::init("Fluid");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pVibrationService = pServer->createService(VIBRATION_SERVICE_UUID);
  BLEService *pIMUService = pServer->createService(IMU_SERVICE_UUID);

  // Add IMU characteristics to the IMU service
  // PROPERTY_NOTIFY -> Notify the client when the value changes
  pIMUCharacteristic = pIMUService->createCharacteristic(CHARACTERISTIC_UUID_IMU, BLECharacteristic::PROPERTY_NOTIFY);
  pIMUCharacteristic->addDescriptor(new BLE2902());

  // Add Vibration characteristics to the Vibration service
  // PROPERTY_WRITE -> with Feedback
  // PROPERTY_WRITE_NR -> without Feedback
  pVLeftCharacteristic = pVibrationService->createCharacteristic(CHARACTERISTIC_UUID_VIBRATION_LEFT, BLECharacteristic::PROPERTY_WRITE);
  pVRightCharacteristic = pVibrationService->createCharacteristic(CHARACTERISTIC_UUID_VIBRATION_RIGHT, BLECharacteristic::PROPERTY_WRITE);
  pVLeftCharacteristic->addDescriptor(new BLE2902());
  pVRightCharacteristic->addDescriptor(new BLE2902());
  // Setting Callbacks
  pVLeftCharacteristic->setCallbacks(new VibrationLeftCallback());
  pVRightCharacteristic->setCallbacks(new VibrationRightCallback());

  // Start the services
  pVibrationService->start();
  pIMUService->start();

  // Add the services to advertising
  BLEAdvertising *advertising = pServer->getAdvertising();
  advertising->addServiceUUID(IMU_SERVICE_UUID);
  advertising->addServiceUUID(VIBRATION_SERVICE_UUID);
  // Start advertising
  pServer->startAdvertising();

  Serial.println("Waiting a client connection to notify...");
}

void sendPacket(uint8_t *pPacket)
{
  pIMUCharacteristic->setValue(pPacket, 17);
  pIMUCharacteristic->notify();
  delay(30); // bluetooth stack will go into congestion, if too many packets are sent
}

bool streamQuat(uint8_t (*pQuatData)[8])
{
  if (!deviceConnected)
  {
    Serial.println("Device n/a");
    return false;
  }
  // Each Transmission we send 17 bytes of uint8_t (2 Quaternions)
  // Each Quaternion Value (w,x,y,z) consists of 2 bytes
  // Lower byte comes before the higher one in the array
  // Last byte is either 00 for continuing stream or 01 for new stream
  // 1 stream is 6 quats split into 3 packets
  bool newStream = true;
  uint8_t packet[17];
  for (int i = 0; i < NUMBER_IMUS; i += 2)
  {
    memcpy(packet, pQuatData[i], 8);
    memcpy(packet + 8, pQuatData[i + 1], 8);
    if (newStream)
    {
      packet[16] = 0x01;
      newStream = false;
    }
    else
    {
      packet[16] = 0x00;
    }
    sendPacket(packet);
  }
  return true;
}
