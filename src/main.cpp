#include <Arduino.h>
#include "ble.h"
#include "imu.h"

int delayForBLB = 500; // delay between BLE broadcasts in ms
int timerForMotor1 = 0; 
int timerForMotor2 = 0; // times the motor is active in loop
int vibratioinIntensity = 0; // intensity of the vibration motor (0-500)

void printQuatData(uint8_t quatData[NUMBER_IMUS][8]);
void printQuatDataAsFloat(uint8_t quatData[NUMBER_IMUS][8]);
void vibration(uint8_t motor, uint8_t strength, uint8_t duration);
void checkVibrationMotor();

void setup() {
    Serial.begin(115200);
    delay(1000);
    setupBLE();
    setVibrationCallback(vibration);
    
    
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
    void checkVibrationMotor();
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

void vibration(uint8_t motor, uint8_t strength, uint8_t duration) {
    Serial.print("Motor: ");
    Serial.print(motor);
    Serial.print(" Strength: ");
    Serial.print(strength);
    Serial.print(" Duration: ");
    Serial.println(duration);
    if(motor == 1) {
        timerForMotor1 = duration; // Set the timer for motor 1
    } else if (motor == 2) {
        timerForMotor2 = duration; // Set the timer for motor 2
    } else if (motor == 3) {
        timerForMotor1 = duration; // Set the timer for motor 1
        timerForMotor2 = duration; // Set the timer for motor 2
    } else {
        Serial.println("Invalid motor number. Use 1, 2, or 3.");
        return;
    }
    
    vibratioinIntensity = strength; // Set the intensity of the vibration motor (0-500)
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