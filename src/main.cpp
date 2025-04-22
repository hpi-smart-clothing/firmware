#include <Arduino.h>
#include "ble.h"
#include "imu.h"

int delayForBLB = 500; // delay between BLE broadcasts in ms
int timerForMotor1 = 0; 
int timerForMotor2 = 0; // times the motor is active in loop
int vibratioinIntensity = 0; // intensity of the vibration motor (0-500)

void printQuatData(uint8_t quatData[NUMBER_IMUS][8]);
void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8]);
void vibrationRight();
void vibrationLeft();
void checkVibrationMotor();

void setup() {
    Serial.begin(115200);
    delay(1000);
    setupBLE();
    setVibrationRightCallback(vibrationRight);
    setVibrationLeftCallback(vibrationLeft);
    
    delay(200);
    if(!setupIMUConnection()) {
        Serial.print("IMU n/a");
        while(1);
        }
    delay(5000);
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    Serial.println("setup done");
}

void loop() {
    // quatData[0][0] returns lower byte of quat of first IMU
    // quatData[0][1] returns higher byte

    uint8_t quatData[NUMBER_IMUS][8]; 
    if(loadData(quatData)){
        printQuatDataAsFloat(quatData);
        streamQuat(quatData);
    }
    else {
        Serial.println("no data");
    }
    void checkVibrationMotor()
    delay(delayForBLB - vibratioinIntensity);
    // Check if the timer for motor is active
}

void printQuatData(uint8_t quatData[NUMBER_IMUS][8]) {
    Serial.println(F("Quaternion-Daten (W, X, Y, Z je 2 Bytes):"));
    for (int imu = 0; imu < NUMBER_IMUS; imu++) {
        Serial.print(F("IMU "));
        Serial.print(imu);
        Serial.print(F(": "));

        for (int i = 0; i < 8; i++) {
            if (quatData[imu][i] < 16) Serial.print("0"); // führende Null für Hex
            Serial.print(quatData[imu][i], HEX);
            Serial.print(" ");
        }

        Serial.println(); // neue Zeile für nächste IMU
    }
    Serial.println();
}

void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8]) {
    Serial.println(F("Quaternion-Daten (W, X, Y, Z als float):"));

    for (int imu = 0; imu < NUMBER_IMUS; imu++) {
        Serial.print(F("IMU "));
        Serial.print(imu);
        Serial.print(F(": "));

        int16_t qw = ((int16_t)quatData[imu][1] << 8) | quatData[imu][0];
        int16_t qx = ((int16_t)quatData[imu][3] << 8) | quatData[imu][2];
        int16_t qy = ((int16_t)quatData[imu][5] << 8) | quatData[imu][4];
        int16_t qz = ((int16_t)quatData[imu][7] << 8) | quatData[imu][6];

        float scale = 1.0 / 16384.0;

        Serial.print("W: "); Serial.print(qw * scale, 4); Serial.print(" ");
        Serial.print("X: "); Serial.print(qx * scale, 4); Serial.print(" ");
        Serial.print("Y: "); Serial.print(qy * scale, 4); Serial.print(" ");
        Serial.print("Z: "); Serial.println(qz * scale, 4);
    }

    Serial.println();
}

void vibrationRight() {
    Serial.println("vibrate right");
    timerForMotor1 = 20; // Set the timer for motor 1
    setVibrationIntensity(100); // Set the intensity for motor 1
}

void vibrationLeft() {
    Serial.println("vibrate left");
    timerForMotor2 = 20; // Set the timer for motor 2
    setVibrationIntensity(100); // Set the intensity for motor 2
}

void setVibrationIntensity(int intensity) {
    vibratioinIntensity = intensity;
}

void checkVibrationMotor() {
    if (timerForMotor1 > 0) {
        if(vibratioinIntensity > 0) {
            digitalWrite(MOTOR_1_PIN, LOW); // turn off motor 1 for a short time
            delay(vibratioinIntensity);
        }
        digitalWrite(MOTOR_1_PIN, HIGH); // turn on motor 1
        timerForMotor1--;
    } else {
        digitalWrite(MOTOR_1_PIN, LOW); // turn off motor 1
    }
    if (timerForMotor2 > 0) {
        if(vibratioinIntensity > 0) {
            digitalWrite(MOTOR_2_PIN, LOW); // turn off motor 2 for a short time
            delay(vibratioinIntensity);
        }
        digitalWrite(MOTOR_2_PIN, HIGH); // turn on motor 2
        timerForMotor2--;
    } else {
        digitalWrite(MOTOR_2_PIN, LOW); // turn off motor 2
    }
}