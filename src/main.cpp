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
void printAccelData(sensors_event_t event);
void printMagData(sensors_event_t event);
void printGyroData(sensors_event_t event);
void printQuatData(imu::Quaternion quat);
void printTempData(sensors_event_t event);
void printEulerData(sensors_event_t event);
void printAllData(sensors_event_t event, imu::Quaternion quat, int i);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  for (int i = 0; i < SIZE; i++){
    tcaSelect(IMU_PORTS[i]);
    Serial.print("Status: Initialise IMU on port: "); Serial.println(IMU_PORTS[i]);
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
      Serial.print("Status: Error: Sensor "); Serial.print(i); Serial.println(" has only zeros!");
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
    Serial.print("Status: Error: TCA9548A Error on channel: "); Serial.println(i);
  }
}

void restartSensor(int i) {
  if (!IMUS[i]->begin()) {
    Serial.print("Status: Error: Sensor "); Serial.print(i); Serial.println(" not found!");
  } else {
    IMUS[i]->setExtCrystalUse(true);
    Serial.print("Status: Success: Sensor ");  Serial.print(i);  Serial.println(" successfully restarted.");
    delay(420);
  }
}

bool checkSensorForZeros(imu::Quaternion quat) {
  if (quat.w() == 0.0 && quat.x() == 0.0 && quat.y() == 0.0 && quat.z() == 0.0)
    return true;
  else 
    return false;
}

void printAccelData(sensors_event_t event) {
  Serial.print("Accelerometer: ");
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.println(" m/s^2");
}

void printMagData(sensors_event_t event) {
  Serial.print("Magnetometer: ");
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.println(" uT");
}
void printGyroData(sensors_event_t event) {
  Serial.print("Gyroscope: ");
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.println(" rad/s");
}

void printQuatData(imu::Quaternion quat) {
  Serial.print("Quaternion: ");
  Serial.print("W: "); Serial.print(quat.w()); Serial.print(" ");
  Serial.print("X: "); Serial.print(quat.x()); Serial.print(" ");
  Serial.print("Y: "); Serial.print(quat.y()); Serial.print(" ");
  Serial.print("Z: "); Serial.print(quat.z()); Serial.println();
}

void printTempData(sensors_event_t event) {
  Serial.print("Temperature: "); Serial.print(event.temperature); Serial.println(" C");
}
void printEulerData(sensors_event_t event) {
  Serial.print("Euler: ");
  Serial.print("Heading: "); Serial.print(event.orientation.x); Serial.print(" ");
  Serial.print("Roll: "); Serial.print(event.orientation.y); Serial.print(" ");
  Serial.print("Pitch: "); Serial.print(event.orientation.z); Serial.println(" degrees");
}

void printAllData(sensors_event_t event, imu::Quaternion quat, int i) {
  Serial.print("Data: Begin Data from IMU "); Serial.print(i); Serial.println(": ");
  printAccelData(event);
  printMagData(event);
  printGyroData(event);
  printQuatData(quat);
  printTempData(event);
  printEulerData(event);
  Serial.print("Data: End Data from IMU "); Serial.println(i);
  Serial.println();
}