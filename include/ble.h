#ifndef BLE_H
#define BLE_H
#include <Arduino.h>

//typedef void (*BLECallback)();
typedef void (*BLECallback)(uint8_t vibrationMotor, uint8_t strength, uint8_t duration);

void setupBLE();
void setVibrationCallback(BLECallback callbackFunction);
bool streamQuat(uint8_t (*pQuatData)[8]);

#endif