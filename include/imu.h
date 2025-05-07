#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include "config.h"

bool setupIMUConnection();
std::array<bool, NUMBER_IMUS> loadData(uint8_t pQuatData[NUMBER_IMUS][8]);
int hasTooManyRestarts();

#endif
