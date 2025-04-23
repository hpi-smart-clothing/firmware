#include <Arduino.h>
#include "BluetoothManager.h"
#include "imu.h"
#include "VibrationManager.h"

void printQuatData(uint8_t quatData[NUMBER_IMUS][8]);
void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8]);
void vibrationCallback(size_t size, const VibrationInterval_t* intervals);

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

void printQuatData(uint8_t quatData[NUMBER_IMUS][8])
{
    Serial.println(F("Quaternion-Daten (W, X, Y, Z je 2 Bytes):"));
    for (int imu = 0; imu < NUMBER_IMUS; imu++)
    {
        Serial.print(F("IMU "));
        Serial.print(imu);
        Serial.print(F(": "));

        for (int i = 0; i < 8; i++)
        {
            if (quatData[imu][i] < 16) Serial.print("0"); // führende Null für Hex
            Serial.print(quatData[imu][i], HEX);
            Serial.print(" ");
        }

        Serial.println(); // neue Zeile für nächste IMU
    }
    Serial.println();
}

void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8])
{
    Serial.println(F("Quaternion-Daten (W, X, Y, Z als float):"));

    for (int imu = 0; imu < NUMBER_IMUS; imu++)
    {
        Serial.print(F("IMU "));
        Serial.print(imu);
        Serial.print(F(": "));

        int16_t qw = ((int16_t)quatData[imu][1] << 8) | quatData[imu][0];
        int16_t qx = ((int16_t)quatData[imu][3] << 8) | quatData[imu][2];
        int16_t qy = ((int16_t)quatData[imu][5] << 8) | quatData[imu][4];
        int16_t qz = ((int16_t)quatData[imu][7] << 8) | quatData[imu][6];

        float scale = 1.0 / 16384.0;

        Serial.print("W: ");
        Serial.print(qw * scale, 4);
        Serial.print(" ");
        Serial.print("X: ");
        Serial.print(qx * scale, 4);
        Serial.print(" ");
        Serial.print("Y: ");
        Serial.print(qy * scale, 4);
        Serial.print(" ");
        Serial.print("Z: ");
        Serial.println(qz * scale, 4);
    }

    Serial.println();
}

void vibrationCallback(const size_t size, const VibrationInterval_t* intervals)
{
    vibrationManager->submitVibrationPattern(size, intervals);
}
