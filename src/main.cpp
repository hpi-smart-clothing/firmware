#include <Arduino.h>
#include "BluetoothManager.h"
#include "imu.h"
#include "VibrationManager.h"

void printQuatData(uint8_t quatData[NUMBER_IMUS][8]);
void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8]);
void vibrationCallback(size_t size, const VibrationInterval_t* intervals);
void printValues(uint8_t* quatData);

VibrationManager* vibrationManager;
BluetoothManager* bluetoothManager;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    vibrationManager = new VibrationManager();
    bluetoothManager = new BluetoothManager(&vibrationCallback);

    delay(200);
    if (!setupIMUConnection())
    {
        Serial.print("IMU n/a");
        while (true);
    }
    delay(5000);
    Serial.println("setup done");
}

void loop()
{
    // quatData[0][0] returns lower byte of quat of first IMU
    // quatData[0][1] returns higher byte

    uint8_t quatData[NUMBER_IMUS][8];
    if (loadData(quatData))
    {
        printQuatDataAsFloat(quatData);
        bluetoothManager->streamIMUQuats(quatData);
        delay(30);
    }
    else
    {
        Serial.println("no data");
    }
}

void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8])
{
    for (int imu = 0; imu < NUMBER_IMUS; imu++)
    {
        Serial.print("Data: Sensor " + String(imu) + ": ");
        printValues(quatData[imu]);
    }
}

void printValues(uint8_t* quatData){


    int16_t qw = ((int16_t)quatData[1] << 8) | (int16_t)quatData[0];
    int16_t qx = ((int16_t)quatData[3] << 8) | (int16_t)quatData[2];
    int16_t qy = ((int16_t)quatData[5] << 8) | (int16_t)quatData[4];
    int16_t qz = ((int16_t)quatData[7] << 8) | (int16_t)quatData[6];
    float scale = 1.0 / 32767.0;
    Serial.print(qw * scale, 4);
    Serial.print(",");
    Serial.print(qx * scale, 4);
    Serial.print(",");
    Serial.print(qy * scale, 4);
    Serial.print(",");
    Serial.println(qz * scale, 4);
  }
  

void vibrationCallback(const size_t size, const VibrationInterval_t* intervals)
{
    vibrationManager->submitVibrationPattern(size, intervals);
}
