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
const uint8_t SIZE = sizeof(IMU_PORTS)/sizeof(*IMU_PORTS);
Adafruit_BNO055 IMUS[SIZE] = { 
  Adafruit_BNO055(55, BNO055_I2C_ADDR),
  Adafruit_BNO055(54, BNO055_I2C_ADDR),
  Adafruit_BNO055(53, BNO055_I2C_ADDR),
  Adafruit_BNO055(52, BNO055_I2C_ADDR),
  Adafruit_BNO055(51, BNO055_I2C_ADDR),
  Adafruit_BNO055(50, BNO055_I2C_ADDR)
};
static unsigned long lastSample = 0;

void tcaSelect(uint8_t i);
bool restartSensor(int i);
void printAllData(sensors_event_t &event, imu::Quaternion &quat, int i);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //Wire.setClock(400000); 

  StaticJsonDocument<128> doc;
  
  for (int i = 0; i < SIZE; i++) {
    tcaSelect(IMU_PORTS[i]);
    doc["i"] = i;
    doc["m"] = String("S: Init: ") + String(IMU_PORTS[i]);
    serializeJson(doc, Serial);
    Serial.println();
    
    if (restartSensor(i)) {
      delay(400);
    }
  }
}

void loop() {
  sensors_event_t event;
  imu::Quaternion quat;
  StaticJsonDocument<256> doc;

  for (int i = 0; i < SIZE; i++) {
    if (i == 2) continue;
    tcaSelect(IMU_PORTS[i]);
    
    IMUS[i].getEvent(&event);
    quat = IMUS[i].getQuat();
    
    if (quat.w() == 0.0 && quat.x() == 0.0 && quat.y() == 0.0 && quat.z() == 0.0) {
      doc["i"] = i;
      doc["m"] = "E: 0s";
      serializeJson(doc, Serial);
      Serial.println();
      restartSensor(i);
    } else {
      printAllData(event, quat, i);
    }
    delay(2);
  }
  wait();
}

void wait() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastSample;
  if (elapsed < BNO055_SAMPLERATE_DELAY_MS) {
    delay(BNO055_SAMPLERATE_DELAY_MS - elapsed);
  }
  lastSample = millis();
}

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  uint8_t result = Wire.endTransmission();
  
  if (result != 0) {
    StaticJsonDocument<64> doc;
    doc["i"] = -1;
    doc["m"] = "E: WE";
    serializeJson(doc, Serial);
    Serial.println();
  }
}

bool restartSensor(int i) {
  if (!IMUS[i].begin()) {
    StaticJsonDocument<64> doc;
    doc["i"] = i;
    doc["m"] = "E: NF";
    serializeJson(doc, Serial);
    Serial.println();
    return false;
  } else {
    IMUS[i].setExtCrystalUse(true);
    return true;
  }
}

void printAllData(sensors_event_t &event, imu::Quaternion &quat, int i) {
  StaticJsonDocument<256> doc;
  JsonArray data = doc.createNestedArray("m");
  
  doc["i"] = i;
  
  data.add(event.acceleration.x);
  data.add(event.acceleration.y);
  data.add(event.acceleration.z);
  data.add(event.magnetic.x);
  data.add(event.magnetic.y);
  data.add(event.magnetic.z);
  data.add(event.gyro.x);
  data.add(event.gyro.y);
  data.add(event.gyro.z);
  data.add(quat.w());
  data.add(quat.x());
  data.add(quat.y());
  data.add(quat.z());
  
  serializeJson(doc, Serial);
  Serial.println();
}
