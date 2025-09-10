#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

static constexpr uint8_t BNO055_CHIP_ID     = 0x00;
static constexpr uint8_t BNO055_ID_CONST    = 0xA0;
static constexpr uint8_t BNO055_OPR_MODE    = 0x3D;
static constexpr uint8_t BNO055_SYS_TRIGGER = 0x3F;
#define BNO055_SAMPLERATE_DELAY_MS 1000
#define BNO055_I2C_ADDR           0x29
#define TCAADDR                   0x70
#define RESTART_DELAY_MS          650

const uint8_t IMU_PORTS[] = {0, 1, 2, 3, 4, 5};
const uint8_t SIZE = sizeof(IMU_PORTS) / sizeof(*IMU_PORTS);
Adafruit_BNO055 IMUS[SIZE] = {
    Adafruit_BNO055(55, BNO055_I2C_ADDR),
    Adafruit_BNO055(54, BNO055_I2C_ADDR),
    Adafruit_BNO055(53, BNO055_I2C_ADDR),
    Adafruit_BNO055(52, BNO055_I2C_ADDR),
    Adafruit_BNO055(51, BNO055_I2C_ADDR),
    Adafruit_BNO055(50, BNO055_I2C_ADDR)
};

static unsigned long lastSample;
unsigned long packetCounter = 0;
// -1: kein Restart aktiv, sonst Zeitpunkt des Restart-Aufrufs
long restartTimestamps[SIZE];

// Prototypen
bool  checkForSample(unsigned long &lastSample, const unsigned long sampleRateMs);
void  printOneIMUData(int i, unsigned long n);
void  printOneIMUZeroData(int i, unsigned long n);
void  printAllIMUData(unsigned long n);
void  tcaSelect(uint8_t i);
bool  restartSensor(int i);
void  triggerReset(int i);
void  finishRestart(int i);
bool  isQuaternionZero(const imu::Quaternion &quat);

void setup() {
    Serial.begin(115200);
    Wire.begin(8, 20);

    StaticJsonDocument<128> doc;
    for (int i = 0; i < SIZE; i++) {
        restartTimestamps[i] = -1;
        tcaSelect(IMU_PORTS[i]);
        Wire.requestFrom(BNO055_I2C_ADDR, 1);
        if (Wire.available() == 0) {
            Serial.printf("no device at 0x29 on chan %u\n", IMU_PORTS[i]);
        }
        doc.clear();
        doc["i"] = i;
        doc["m"] = String("S: Init: ") + String(IMU_PORTS[i]);
        serializeJson(doc, Serial);
        Serial.println();

        // klassischer Blocking-Init für Erst-Konfiguration
        restartSensor(i);
    }
    lastSample = millis();
}

void loop() {
    if (checkForSample(lastSample, BNO055_SAMPLERATE_DELAY_MS)) {
        printAllIMUData(packetCounter);
        packetCounter++;
        if (packetCounter >= 100000) {
            ESP.restart();
        }
    }
}

bool checkForSample(unsigned long &lastSample, const unsigned long sampleRateMs) {
    unsigned long now = millis();
    if (now - lastSample >= sampleRateMs) {
        lastSample += sampleRateMs;
        return true;
    }
    return false;
}

void printOneIMUData(int i, unsigned long n) {
    StaticJsonDocument<256> doc;
    JsonArray data = doc.createNestedArray("m");
    doc["i"] = i;
    doc["t"] = millis();
    doc["n"] = n;
    Adafruit_BNO055 &imu = IMUS[i];

    // Sensordaten sammeln
    auto acc  = imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    auto lin  = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    auto grav = imu.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    auto mag  = imu.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    auto gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    auto quat = imu.getQuat();

    // Daten hinzufügen
    data.add(acc.x());  data.add(acc.y());  data.add(acc.z());
    data.add(lin.x());  data.add(lin.y());  data.add(lin.z());
    data.add(grav.x()); data.add(grav.y()); data.add(grav.z());
    data.add(mag.x());  data.add(mag.y());  data.add(mag.z());
    data.add(gyro.x()); data.add(gyro.y()); data.add(gyro.z());
    data.add(quat.w()); data.add(quat.x()); data.add(quat.y()); data.add(quat.z());

    serializeJson(doc, Serial);
    Serial.println();
}

void printOneIMUZeroData(int i, unsigned long n) {
    StaticJsonDocument<256> doc;
    JsonArray data = doc.createNestedArray("m");
    doc["i"] = i;
    doc["t"] = millis();
    doc["n"] = n;
    // 3+3+3+3+3+4 = 19 Werte
    for (int v = 0; v < 19; v++) {
        data.add(0);
    }
    serializeJson(doc, Serial);
    Serial.println();
}

void printAllIMUData(unsigned long n) {
    for (int i = 0; i < SIZE; i++) {
        if (i == 2) continue;  // optional überspringen

        tcaSelect(IMU_PORTS[i]);

        // Restart asynchron behandeln
        if (restartTimestamps[i] != -1) {
            if (millis() - restartTimestamps[i] < RESTART_DELAY_MS) {
                printOneIMUZeroData(i, n);
            } else {
                finishRestart(i);
            }
            continue;
        }

        // Quat pro Sensor prüfen
        imu::Quaternion quat = IMUS[i].getQuat();
        if (isQuaternionZero(quat)) {
            triggerReset(i);
            printOneIMUZeroData(i, n);
        } else {
            printOneIMUData(i, n);
        }
    }
}

void tcaSelect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

bool restartSensor(int i) {
    if (IMUS[i].begin()) {
        IMUS[i].setMode(OPERATION_MODE_NDOF);
        delay(20);
        IMUS[i].setExtCrystalUse(true);
        delay(20);
        return true;
    }
    return false;
}

void triggerReset(int i) {
    tcaSelect(IMU_PORTS[i]);
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(0x3F);    // SYS_TRIGGER
    Wire.write(0x01);    // RST_SYS = 1
    Wire.endTransmission();
    restartTimestamps[i] = millis();
}

void finishRestart(int i) {
  tcaSelect(IMU_PORTS[i]);

  // 1) Einmaliger Chip-ID-Check
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(BNO055_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_I2C_ADDR, (uint8_t)1);
  if (!(Wire.available() && Wire.read() == BNO055_ID_CONST)) {
    // Sensor noch nicht boot-bereit → nochmal 650 ms warten
    restartTimestamps[i] = millis();
    return;
  }

  // 2) Jetzt sicher ready → in CONFIGMODE schalten
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(BNO055_OPR_MODE);
  Wire.write(0x00);  // CONFIGMODE
  Wire.endTransmission();
  delay(10);         // ≥7 ms gemäß Datenblatt

  // 3) NDOF-Modus
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(BNO055_OPR_MODE);
  Wire.write(OPERATION_MODE_NDOF);
  Wire.endTransmission();
  delay(10);

  // 4) Externen Kristall aktivieren
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(BNO055_SYS_TRIGGER);
  Wire.write(0x80);  // CLK_SEL = 1
  Wire.endTransmission();

  // 5) Fertig – Sensor wieder freigeben
  restartTimestamps[i] = -1;
}

bool isQuaternionZero(const imu::Quaternion &quat) {
    return quat.w() == 0.0 && quat.x() == 0.0 && quat.y() == 0.0 && quat.z() == 0.0;
}