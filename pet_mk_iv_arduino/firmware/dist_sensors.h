#ifndef _PET_DISTSENSORS_H
#define _PET_DISTSENSORS_H

#include <Arduino.h>

#include <stdint.h>

void distSensorSetup();
void distSensorUpdate();

void echoCheck();
void publishResult(uint8_t sensor, int16_t dist);

#endif
