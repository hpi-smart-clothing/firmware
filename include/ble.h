#ifndef BLE_H
#define BLE_H
#include <Arduino.h>

typedef void (*BLECallback)();

void setupBLE();
void setVibrationRightCallback(BLECallback callbackFunction);
void setVibrationLeftCallback(BLECallback callbackFunction);
bool streamQuat(uint8_t (*pQuatData)[8]);

#endif