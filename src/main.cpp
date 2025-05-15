#include <Arduino.h>
#include "BluetoothManager.h"
#include "imu.h"
#include "VibrationManager.h"

void printQuatData(uint8_t quatData[NUMBER_IMUS][8]);
void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8]);
void vibrationCallback(size_t size, const VibrationInterval_t* intervals);
void printValues(uint8_t* quatData);
int getBatteryLevel();
int readVoltage();
int lookupPercent(float measuredVoltage);

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
    delay(REQUIERD_TIME_AFTER_IMU_START);
    Serial.println("setup done");
}

void loop()
{
    // quatData[0][0] returns lower byte of quat of first IMU
    // quatData[0][1] returns higher byte

    uint8_t quatData[NUMBER_IMUS][8];
    std::array<bool, NUMBER_IMUS> status = loadData(quatData);
    bool allFalse = true;
    for(bool v: status)
    {
        if (v) allFalse = false;
    }
    if (allFalse)
    {
        Serial.println("Status: Error: no data");
    }
    else
    {
        int sensorIdx = hasTooManyRestarts();
        if(sensorIdx!=NUMBER_IMUS+1)
        {
            Serial.println("Status: Error: no data even after 3 restarts in Sensor: " + String(sensorIdx)); 
            Serial.println("Status: Restarting ESP");
            delay(200);
            ESP.restart();
        }
        else
        {
            printQuatDataAsFloat(quatData);
            bluetoothManager->streamIMUQuats(quatData);
        }
    }
    Serial.println("Status: Battery: " + String(getBatteryLevel()) + "%");
    Serial.println("Status: Voltage: " + String(readVoltage()) + "V");
    delay(SAMPLE_FREQUENCY);
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

int readVoltage()
{
    float rawValue = analogRead(READ_VOLTAGE_PIN);
    float voltage = rawValue * float(ADC_VREF) / float(ADCRESOLUTION);
    voltage = voltage * (float(READ_VOLTAGE_RESISTOR1) + float(READ_VOLTAGE_RESISTOR2)) / float(READ_VOLTAGE_RESISTOR2);
    return voltage;
}

int lookupPercent(float measuredVoltage) {
    for (int i = 0; i < sizeof(VoltageLevel) / sizeof(VoltageEntry); i++) {
        if (measuredVoltage >= VoltageLevel[i].mesuredVoltage) {
        return VoltageLevel[i].percentage;
        }
    }
    return 0; // Return 0% if no match is found
}
int getBatteryLevel() {
  float measuredVoltage = readVoltage();
  int percentage = lookupPercent(measuredVoltage);
  return percentage;
}