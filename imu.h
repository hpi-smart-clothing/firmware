#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include "config.h"

bool setupIMUConnection();
bool loadData(uint8_t pQuatData[NUMBER_IMUS][8]);

#endif
