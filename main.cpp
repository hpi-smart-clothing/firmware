#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BNO055_I2C_ADDR 0x29 
#define TCAADDR 0x70  

const int IMU_PORTS[] = {0, 2}; //INSERT PORTS HERE
const int SIZE = sizeof(IMU_PORTS)/sizeof(*IMU_PORTS);
Adafruit_BNO055 IMUS[SIZE]; 

void printValues(imu::Quaternion quat);
void tcaSelect(uint8_t i);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    Adafruit_BNO055 bno055 = Adafruit_BNO055(55 - i, BNO055_I2C_ADDR);
    IMUS[i] = bno055;
    if (!IMUS[i].begin()) {
      Serial.println("Sensor nicht gefunden!");
    }
  }
}

void loop() {
  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    imu::Quaternion quat = IMUS[i].getQuat();
    if(i != 0)
      Serial.print(",");
    printValues(quat);
  }
  Serial.println();
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
  if (i > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}