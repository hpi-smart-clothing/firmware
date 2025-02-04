#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <math.h>
#include <BluetoothSerial.h>
#include <BLE2902.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BNO055_I2C_ADDR 0x29
#define TCAADDR 0x70

#define SERVICE_UUID "e4ae6494-1b35-4bb8-99a6-8ab8f3e699f1"
#define CHARACTERISTIC_UUID "441d3fa0-67e1-4fbe-acd1-4a7b004def82"

bool shouldSend = false;

const int IMU_PORTS[] = {0, 2}; // INSERT PORTS HERE
const int SIZE = sizeof(IMU_PORTS) / sizeof(*IMU_PORTS);
Adafruit_BNO055 IMUS[SIZE];

BLECharacteristic *pCharacteristic;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    shouldSend = true;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
  {
    shouldSend = true;
  }
};

void tcaSelect(uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  for (int i = 0; i < SIZE; i++)
  {
    tcaSelect(IMU_PORTS[i]);
    Adafruit_BNO055 bno055 = Adafruit_BNO055(55 - i, BNO055_I2C_ADDR);
    IMUS[i] = bno055;
    if (!IMUS[i].begin())
    {
      Serial.println("Sensor nicht gefunden!");
    }
  }

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void printValues(imu::Quaternion quat)
{
  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.println(quat.z());
}

void loop()
{
  char buffers[SIZE][50];
  for (int i = 0; i < SIZE; i++)
  {
    tcaSelect(IMU_PORTS[i]);
    imu::Quaternion quat = IMUS[i].getQuat();
    sprintf(buffers[i], "IMU %d:%3.2f,%3.2f,%3.2f,%3.2f\n", i, quat.w(), quat.x(), quat.y(), quat.z());
    //printValues(quat);
  }


  if (shouldSend)
  {
    char payload[50*SIZE];
    strcpy(payload, buffers[0]);
    for (int i = 1; i < SIZE; i++){
      strcat(payload, buffers[i]);
    }
    Serial.println(payload);
    pCharacteristic->setValue(payload);
    pCharacteristic->notify();
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
