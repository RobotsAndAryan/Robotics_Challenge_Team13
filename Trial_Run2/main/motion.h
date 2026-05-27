#pragma once
void setMotors(int l, int r, int max_pwm);
void stopMotors();
void turnAngle(float targetAngle, bool turnLeft);
void moveForwardTicks(long targetTicks);
void moveStraightDeadReckoning(long targetTicks);