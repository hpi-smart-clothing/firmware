#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BNO055_I2C_ADDR 0x29 
#define TCAADDR 0x70  

const int IMU_PORTS[] = {0, 1, 2, 3, 4, 5}; //INSERT PORTS HERE
const int SIZE = sizeof(IMU_PORTS)/sizeof(*IMU_PORTS);
Adafruit_BNO055* IMUS[SIZE]; 

void printValues(imu::Quaternion quat);
void tcaSelect(uint8_t i);

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();
  delay(500);

  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    Serial.print("Status: Initialisierung: Initialise IMU in port: "); Serial.println(IMU_PORTS[i]);
    delay(100);
    IMUS[i] = new Adafruit_BNO055(55 - i, BNO055_I2C_ADDR);
    if (!IMUS[i]->begin()) {
      Serial.print("Status: Error: Sensor "); Serial.print(i); Serial.println(" not found!");
    } else {
      IMUS[i]->setExtCrystalUse(true);
    }
    delay(100);
  }
  delay(1000);
}

void loop() {
  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    //checkSensorForError(i);
    imu::Quaternion quat = IMUS[i]->getQuat();
    checkSensorForZeros(quat, i);
    Serial.println();
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printValues(imu::Quaternion quat){
  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.print(quat.z());
}

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("Status: Error: TCA9548A Error on channel: "); Serial.println(i);
  }
}

void checkSensorForError(int i) {
  uint8_t system_status, self_test_result, system_error;
  IMUS[i]->getSystemStatus(&system_status, &self_test_result, &system_error);
  if (system_status == 0 || system_error != 0) {
    Serial.print("Status: Error: Error recognised! Status: ");
    Serial.print(system_status);
    Serial.print(", Errorcode: ");
    Serial.println(system_error);
    restartSensor(i);
  }
}

void restartSensor(int i) {
  if (!IMUS[i]->begin()) {
      Serial.print("Status: Error: Sensor "); Serial.print(i); Serial.println(" not found!");
    } else {
      IMUS[i]->setExtCrystalUse(true);
      Serial.print("Status: Success: Sensor ");  Serial.print(i);  Serial.println(" successfully restarted.");
      delay(50);
    }
}

void checkSensorForZeros(imu::Quaternion quat, int i) {
  if (quat.w() == 0.0 && quat.x() == 0.0 && quat.y() == 0.0 && quat.z() == 0.0) {
    Serial.print("Status: Error: Sensor "); Serial.print(i); Serial.print(": Quaternion contains only zeros!");
    restartSensor(i);
  }
  else  {
    Serial.print("Data: Sensor ");  Serial.print(i); Serial.print(": ");
    printValues(quat);
  }
}
