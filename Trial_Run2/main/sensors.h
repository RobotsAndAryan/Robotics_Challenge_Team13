#pragma once
#include <Wire.h>
int getLidar(TwoWire &w, int addr);
void checkFrontObstacle();
float getPitch();
int getFrontClearanceMM();
