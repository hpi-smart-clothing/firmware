#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BNO055_I2C_ADDR 0x29 // Sicherstellen, dass ADR mit VCC verbunden ist
#define TCAADDR 0x70         // Adresse des Multiplexers

Adafruit_BNO055 bno055_1 = Adafruit_BNO055(55, BNO055_I2C_ADDR);
Adafruit_BNO055 bno055_2 = Adafruit_BNO055(54, BNO055_I2C_ADDR);
//Adafruit_BNO055 bno055_2 = Adafruit_BNO055(55);
// Fügen Sie weitere Objekte für Ihre Sensoren hinzu.
const int IMU_PORTS[] = {2, 0};
const Adafruit_BNO055 IMUS[] = {bno055_2, bno055_1};

void tcaSelect(uint8_t i) {
  if (i > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  // Sensor 1 initialisieren
  tcaSelect(0);
  if (!bno055_1.begin())
  {
    Serial.println("Sensor 0 nicht gefunden!");
  }

  tcaSelect(2);
  if (!bno055_1.begin())
  {
    Serial.println("Sensor 2 nicht gefunden!");
  }
}

void printValues(imu::Quaternion quat){
  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.println(quat.z());
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i = 0; i < sizeof(IMU_PORTS)/sizeof(*IMU_PORTS); i++){
    tcaSelect(IMU_PORTS[i]);
    Adafruit_BNO055 imu = IMUS[i];
    imu::Quaternion quat = imu.getQuat();
    printValues(quat);
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
