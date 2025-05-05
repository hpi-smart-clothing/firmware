#include "BluetoothManager.h"
#include <BLE2902.h>
#include "config.h"
#include <Arduino.h>

#define IMU_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define VIBRATION_SERVICE_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_VIBRATION "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_IMU "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"

BluetoothManager::BluetoothManager(void (*vibrationCallback)(size_t size, const VibrationInterval_t *intervals))
    : vibrationCallback(vibrationCallback)
{
    BLEDevice::init(BLE_DEVICE_NAME);

    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new ServerCallbacks);

    BLEService* pVibrationService = pServer->createService(VIBRATION_SERVICE_UUID);
    BLEService* pIMUService = pServer->createService(IMU_SERVICE_UUID);

    pIMUCharacteristic = pIMUService->createCharacteristic(CHARACTERISTIC_UUID_IMU, BLECharacteristic::PROPERTY_NOTIFY);
    pIMUCharacteristic->addDescriptor(new BLE2902());

    pVibrationCharacteristic = pVibrationService->createCharacteristic(CHARACTERISTIC_UUID_VIBRATION, BLECharacteristic::PROPERTY_WRITE);
    pVibrationCharacteristic->addDescriptor(new BLE2902());

    pVibrationCharacteristic->setCallbacks(new VibrationCharacteristicCallbacks(this));

    pVibrationService->start();
    pIMUService->start();

    BLEAdvertising* advertising = pServer->getAdvertising();
    advertising->addServiceUUID(IMU_SERVICE_UUID);
    advertising->addServiceUUID(VIBRATION_SERVICE_UUID);
    pServer->startAdvertising();
}

void BluetoothManager::streamIMUQuats(uint8_t (*pQuatData)[8]) const
{
    // send all quaternions in one packet, 48 bytes, 2 bytes per quat component in fixed point
    bool newStream = true;
    uint8_t packet[48];
    for (int i = 0; i < NUMBER_IMUS; i += 1)
    {
        memcpy(packet+i*8, pQuatData[i], 8);
    }
    pIMUCharacteristic->setValue(packet, 48);
    pIMUCharacteristic->notify();
}

void BluetoothManager::streamIMUQuats2(uint8_t (*pQuatData)[8]) const
{
    // Transmits 6 quaternions in one 48 byte packet at once

    uint8_t quatStream[48];
    
    for(int i = 0; i < NUMBER_IMUS; i ++)
    {
        memcpy(quatStream + 8 * i, pQuatData[i], 8);
    }

    pIMUCharacteristic->setValue(quatStream, 48);
    pIMUCharacteristic->notify();
    
}

void BluetoothManager::handleVibrationData(size_t size, uint8_t* data) const
{
    const auto intervals = new VibrationInterval_t[size / 4];
    for (int i = 0; i < size / 4; i++)
    {
        const auto interval = &intervals[i];
        interval->duration = data[4 * i + 1] << 8 | data[4 * i];
        interval->leftIntensity = data[4 * i + 2];
        interval->rightIntensity = data[4 * i + 3];
        Serial.println(interval->duration);
    }
    vibrationCallback(size / 4, intervals);
    delete[] intervals;
}


void BluetoothManager::ServerCallbacks::onConnect(BLEServer* pServer)
{
    Serial.println("Device connected");
}

void BluetoothManager::ServerCallbacks::onDisconnect(BLEServer* pServer)
{
    Serial.println("Device disconnected");
}

void BluetoothManager::VibrationCharacteristicCallbacks::onWrite(BLECharacteristic* pCharacteristic)
{
    manager->handleVibrationData(pCharacteristic->getLength(), pCharacteristic->getData());
}