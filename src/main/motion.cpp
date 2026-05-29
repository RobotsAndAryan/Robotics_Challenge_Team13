// motion.cpp - low-level motion primitives (turns, straight drives, speed control)
#include "config.h"
#include "motion.h"
#include "sensors.h"
#include <Arduino.h>

// clamps speeds to max_pwm and sends to motoron - negated because of our motor wiring
void setMotors(int l, int r, int max_pwm) {
  if(l > max_pwm) l = max_pwm;
  if(l < -max_pwm) l = -max_pwm;
  if(r > max_pwm) r = max_pwm;
  if(r < -max_pwm) r = -max_pwm;
  mc.setSpeed(1, -l);
  mc.setSpeed(3, -r);
}

void stopMotors() {
  setMotors(0, 0, 800);
}

// gyro-based turning - integrates angular velocity to track how far we've rotated
// 12 degree undershoot compensates for momentum/inertia (tuned on the actual robot)
// 300ms delays before and after let the robot settle so gyro readings are clean
void turnAngle(float targetAngle, bool turnLeft) {
  stopMotors();
  unsigned long dStart = millis();
  while(millis() - dStart < 300) { updateUI(); if(!robotEnabled()) return; delay(1); }

  float currentYaw = 0;
  unsigned long lastIMUTime = micros();
  int lSpeed = turnLeft ? -turning_spd : turning_spd;
  int rSpeed = turnLeft ? turning_spd : -turning_spd;

  setMotors(lSpeed, rSpeed, 800);
  float actualTarget = targetAngle - 12.0;
  unsigned long turnStart = millis();

  // integrate gyro Z until we've rotated enough (or 4s timeout as safety)
  while(abs(currentYaw) < actualTarget) {
    updateUI();
    if(!robotEnabled()) { stopMotors(); return; }
    if(millis() - turnStart > 4000) { break; }

    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;
    // subtract bias calibrated at startup, convert rad/s to deg/s
    float gyroZ = (g.gyro.z - z_bias) * 57.2958;
    // deadband filter - ignore tiny readings that are just noise
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;
  }

  stopMotors();
  dStart = millis();
  while(millis() - dStart < 300) { updateUI(); if(!robotEnabled()) return; delay(1); }
  lastError = 0;
}

// encoder-based forward movement - stops when EITHER encoder hits target
// AND logic means we stop as soon as the first wheel gets there (prevents overshoot)
void moveForwardTicks(long targetTicks) {
  pos1 = 0; pos2 = 0;
  unsigned long moveStart = millis();
  setMotors(baseSpeed_6V, baseSpeed_6V, 440);
  while(abs(pos1) < targetTicks && abs(pos2) < targetTicks) {
    updateUI();
    if(!robotEnabled()) { stopMotors(); return; }
    if(millis() - moveStart > 5000) { break; }
    delay(1);
  }
  stopMotors();
}

// drives straight using gyro to correct drift - for when there's no line to follow
// P-controller on heading error keeps us pointing the same direction we started
void moveStraightDeadReckoning(long targetTicks) {
  pos1 = 0; pos2 = 0;
  float currentYaw = 0;
  unsigned long lastIMUTime = micros();
  unsigned long moveStart = millis();

  while(abs(pos1) < targetTicks && abs(pos2) < targetTicks) {
    updateUI();
    if(!robotEnabled()) { stopMotors(); return; }
    if(millis() - moveStart > 5000) { break; }

    // check for obstacles even during dead reckoning
    checkFrontObstacle();
    if(pathBlocked) { stopMotors(); return; }

    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;

    float gyroZ = (g.gyro.z - z_bias) * 57.2958;
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;

    // correct towards 0 yaw (i.e. keep going straight)
    float headingError = 0.0 - currentYaw;
    float correction = Kp_heading * headingError;

    setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);
    delay(2);
  }
  stopMotors();
}