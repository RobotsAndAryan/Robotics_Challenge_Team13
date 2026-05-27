#pragma once
#include <Wire.h>

void setMotors(int l, int r, int max_pwm);
void stopMotors();
void turnAngle(float targetAngle, bool turnLeft);
void moveForwardTicks(long targetTicks);
void moveStraightDeadReckoning(long targetTicks);
// NEW: Dynamic LiDAR edge detection
void moveStraightUntilLidarClears(TwoWire &w, int addr, int clearThreshold);