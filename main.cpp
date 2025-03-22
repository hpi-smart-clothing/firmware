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
#include <stdlib.h>

#define BNO055_SAMPLERATE_DELAY_MS (1000)
#define BNO055_I2C_ADDR 0x29
#define TCAADDR 0x70

#define SERVICE_UUID "e4ae6494-1b35-4bb8-99a6-8ab8f3e699f1"
#define QUATERNIONS_UUID "441d3fa0-67e1-4fbe-acd1-4a7b004def82"
#define VIBRATION_UUID "2b0949a4-4851-46fd-a53b-0db7a95c499b"

bool shouldSend = false;
bool sensorsAttached = true;

const int IMU_PORTS[] = {0, 1, 2, 3, 4}; // INSERT PORTS HERE
const int SIZE = sizeof(IMU_PORTS) / sizeof(*IMU_PORTS);
Adafruit_BNO055 IMUS[SIZE];

BLECharacteristic *pQuatCharacteristic;
BLECharacteristic *pVibrationCharacteristic;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    shouldSend = true;
    Serial.println("Es druckt!");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
  {
    shouldSend = true;
  }

  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param){
    if (pCharacteristic->getUUID().toString() == "2b0949a4-4851-46fd-a53b-0db7a95c499b"){
      Serial.println((char *)(pCharacteristic->getData()));
    }
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
      sensorsAttached = false;
    }
  }

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pQuatCharacteristic = pService->createCharacteristic(
      QUATERNIONS_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pQuatCharacteristic->addDescriptor(new BLE2902());
  pQuatCharacteristic->setCallbacks(new MyCallbacks());
  pQuatCharacteristic->setValue("No Quaternions Sent");

  pVibrationCharacteristic = pService->createCharacteristic(
      VIBRATION_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pVibrationCharacteristic->addDescriptor(new BLE2902());
  pVibrationCharacteristic->setCallbacks(new MyCallbacks());
  pVibrationCharacteristic->setValue("No Vibration Messages Recieved");

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
    if (sensorsAttached){
      tcaSelect(IMU_PORTS[i]);
      imu::Quaternion quat = IMUS[i].getQuat();
      sprintf(buffers[i], "IMU %d:%3.2f,%3.2f,%3.2f,%3.2f\n", i, quat.w(), quat.x(), quat.y(), quat.z());
    }else{
      // Fallback Demo Mode when IMUs are not Attached
      float offset = (1.0 / ((std::rand() % 10) + 0.1));
      sprintf(buffers[i], "IMU %d:%3.2f,%3.2f,%3.2f,%3.2f\n", i, 0.333 + offset, 0.444 + offset, 0.555 + offset, 0.666 + offset);
    }
  }


  if (shouldSend)
  {
    char payload[50*SIZE];
    strcpy(payload, buffers[0]);
    for (int i = 1; i < SIZE; i++){
      strcat(payload, buffers[i]);
    }
    pQuatCharacteristic->setValue(payload);
    pQuatCharacteristic->notify();
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
