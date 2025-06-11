#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

#define BNO055_SAMPLERATE_DELAY_MS (99)
#define BNO055_I2C_ADDR 0x29
#define TCAADDR 0x70

const uint8_t IMU_PORTS[] = {0, 1, 2, 3, 4, 5};
const uint8_t SIZE = sizeof(IMU_PORTS) / sizeof(*IMU_PORTS);
Adafruit_BNO055 IMUS[SIZE] = {
    Adafruit_BNO055(55, BNO055_I2C_ADDR),
    Adafruit_BNO055(54, BNO055_I2C_ADDR),
    Adafruit_BNO055(53, BNO055_I2C_ADDR),
    Adafruit_BNO055(52, BNO055_I2C_ADDR),
    Adafruit_BNO055(51, BNO055_I2C_ADDR),
    Adafruit_BNO055(50, BNO055_I2C_ADDR)};
static unsigned long lastSample = 0;

void tcaSelect(uint8_t i);
bool restartSensor(int i);
void printAllData(imu::Quaternion &quat, int i);
void wait();

void setup()
{
  Serial.begin(9600);
  Wire.begin(8, 20);

  StaticJsonDocument<128> doc;

  for (int i = 0; i < SIZE; i++)
  {
    tcaSelect(IMU_PORTS[i]);
    Wire.requestFrom(0x29, 1);
    if (Wire.available() == 0)
    {
      Serial.printf("no device at 0x29 on chan %u\n", IMU_PORTS[i]);
    }
    doc["i"] = i;
    doc["m"] = String("S: Init: ") + String(IMU_PORTS[i]);
    serializeJson(doc, Serial);
    Serial.println();

    restartSensor(i);
  }
}

void loop()
{
  sensors_event_t event;
  imu::Quaternion quat;
  StaticJsonDocument<256> doc;
  for (int i = 0; i < SIZE; i++)
  {
    if (i == 2)
      continue;
    tcaSelect(IMU_PORTS[i]);
    delay(10);

    IMUS[i].getEvent(&event);
    quat = IMUS[i].getQuat();

    if (quat.w() == 0.0 && quat.x() == 0.0 && quat.y() == 0.0 && quat.z() == 0.0)
    {
      restartSensor(i);
    }
    printAllData(quat, i);
    delay(2);
  }
  wait();
}

void wait()
{
  unsigned long now = millis();
  unsigned long elapsed = now - lastSample;
  if (elapsed < BNO055_SAMPLERATE_DELAY_MS)
  {
    delay(BNO055_SAMPLERATE_DELAY_MS - elapsed);
  }
  lastSample = millis();
}

void tcaSelect(uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  uint8_t result = Wire.endTransmission();
}

bool restartSensor(int i)
{
  if (IMUS[i].begin())
  {
    delay(100);
    IMUS[i].setMode(OPERATION_MODE_NDOF);
    delay(100);
    IMUS[i].setExtCrystalUse(true);
    delay(100);
    delay(100);
    return true;
  }
  return false;
  return false;
}

void printAllData(imu::Quaternion &quat, int i)
{
  StaticJsonDocument<256> doc;
  JsonArray data = doc.createNestedArray("m");
  doc["i"] = i;
  Adafruit_BNO055& current_imu = IMUS[i];

  // Accelerometer (m/s^2)
  imu::Vector<3> acc = current_imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data.add(acc.x());
  data.add(acc.y());
  data.add(acc.z());

  // Linear Acceleration (m/s^2)
  imu::Vector<3> lin = current_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  data.add(lin.x());
  data.add(lin.y());
  data.add(lin.z());   

  // Gravity (m/s^2)
  imu::Vector<3> grav = current_imu.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  data.add(grav.x());
  data.add(grav.y());
  data.add(grav.z()); 

  // Magnetometer (uT)
  imu::Vector<3> mag = current_imu.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  data.add(mag.x());
  data.add(mag.y());
  data.add(mag.z());

  // Gyroscope (rad/s)
  imu::Vector<3> gyro = current_imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data.add(gyro.x());
  data.add(gyro.y());
  data.add(gyro.z()); 

  data.add(quat.w());
  data.add(quat.x());
  data.add(quat.y());
  data.add(quat.z());

  serializeJson(doc, Serial);
  Serial.println()
}
