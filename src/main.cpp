#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (1000)
#define BNO055_I2C_ADDR 0x29 
#define TCAADDR 0x70  

const int IMU_PORTS[] = {0, 1, 2, 3, 4, 5}; //INSERT PORTS HERE
const int SIZE = sizeof(IMU_PORTS)/sizeof(*IMU_PORTS);
Adafruit_BNO055* IMUS[SIZE]; 

void tcaSelect(uint8_t i);
void restartSensor(int i);
bool checkSensorForZeros(imu::Quaternion quat);
void printAllData(sensors_event_t event, imu::Quaternion quat, int i);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    Serial.print("{")
    Serial.print("\"imu\":"); Serial.print(i); Serial.print(",");
    Serial.print("\"message\":"); Serial.print("\"Status: Initialize IMU on port: "); Serial.print(IMU_PORTS[i]); Serial.print("\"");
    Serial.println("}");
    IMUS[i] = new Adafruit_BNO055(55 - i, BNO055_I2C_ADDR);
    restartSensor(i);
    delay(420);
  }
  delay(420);
}

void loop() {
  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    sensors_event_t event;
    IMUS[i]->getEvent(&event);
    imu::Quaternion quat = IMUS[i]->getQuat();
    if (checkSensorForZeros(quat)) {
      Serial.print("{")
      Serial.print("\"imu\":"); Serial.print(i); Serial.print(",");
      Serial.print("\"message\":"); Serial.print("\"Error: Only Zeros\"");
      Serial.println("}");
      restartSensor(i);
    }
    else
        printAllData(event, quat, i);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("{")
    Serial.print("\"imu\":"); Serial.print(-1); Serial.print(",");
    Serial.print("\"message\":"); Serial.print("\"Error: TCA9548A Error on channel\"");
    Serial.println("}");
  }
}

void restartSensor(int i) {
  if (!IMUS[i]->begin()) {
    Serial.print("{")
    Serial.print("\"imu\":"); Serial.print(i); Serial.print(",");
    Serial.print("\"message\":"); Serial.print("\"Error: Sensor Not Found\"");
    Serial.println("}");
  } else {
    IMUS[i]->setExtCrystalUse(true);
    Serial.print("{")
    Serial.print("\"imu\":"); Serial.print(i); Serial.print(",");
    Serial.print("\"message\":"); Serial.print("\"Status: Sensor Success Restarted\"");
    Serial.println("}");
    delay(420);
  }
}

bool checkSensorForZeros(imu::Quaternion quat) {
  if (quat.w() == 0.0 && quat.x() == 0.0 && quat.y() == 0.0 && quat.z() == 0.0)
    return true;
  else 
    return false;
}

void printAllData(sensors_event_t event, imu::Quaternion quat, int i) {
  Serial.print("{");
  
  Serial.print("\"imu\":"); Serial.print(i); Serial.print(",");

  Serial.print("\"message\":"); Serial.print("\"Data: Success\""); Serial.print(",");

  Serial.print("\"values\":[");
  Serial.print(event.acceleration.x); Serial.print(",");
  Serial.print(event.acceleration.y); Serial.print(",");
  Serial.print(event.acceleration.z); Serial.print(",");
  Serial.print(event.magnetic.x); Serial.print(",");
  Serial.print(event.magnetic.y); Serial.print(",");
  Serial.print(event.magnetic.z); Serial.print(",");
  Serial.print(event.gyro.x); Serial.print(",");
  Serial.print(event.gyro.y); Serial.print(",");
  Serial.print(event.gyro.z); Serial.print(",");
  Serial.print(quat.w()); Serial.print(",");
  Serial.print(quat.x()); Serial.print(",");
  Serial.print(quat.y()); Serial.print(",");
  Serial.print(quat.z()); Serial.print(",");
  Serial.print(event.temperature); Serial.print(",");
  Serial.print(event.orientation.x); Serial.print(",");
  Serial.print(event.orientation.y); Serial.print(",");
  Serial.print(event.orientation.z); Serial.print("]");
  
  Serial.println("}");
}
