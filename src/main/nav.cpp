// nav.cpp - higher-level navigation algorithms (line following, wall following)
#include "config.h"
#include "nav.h"
#include "motion.h"
#include "sensors.h"
#include <Arduino.h>

// line following using RC-timing on reflectance sensors
// charges each sensor pin, then measures how long it takes to discharge through the surface
// dark surface (line) = slower discharge = higher time value
bool executeLineFollow(int bSpeed, int maxPWM) {
  uint16_t lineVals[9];
  // charge all sensor capacitors
  for(int i=0; i<9; i++) { pinMode(linePins[i], OUTPUT); digitalWrite(linePins[i], HIGH); }
  delayMicroseconds(15);
  // switch to input and time the discharge
  for(int i=0; i<9; i++) { pinMode(linePins[i], INPUT); lineVals[i] = 1000; }

  unsigned long st = micros();
  while(micros() - st < 1000) {
    for(int i=0; i<9; i++) {
      if(lineVals[i] == 1000 && digitalRead(linePins[i]) == LOW) lineVals[i] = micros() - st;
    }
  }

  // weighted centroid - sensors closer to centre have higher weight
  // this gives us a signed error value showing which side the line is on
  long num = 0; long den = 0;
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) {
      num += (long)lineVals[i] * weights[i];
      den += lineVals[i];
    }
  }

  if (den == 0) return false; // no line detected at all

  // PD controller - no I term because we don't need to eliminate steady-state error
  // the line is always right there so P+D responds fast enough
  float error = (float)num / den;
  float P = error;
  float D = error - lastError;
  float correction = (Kp_line * P) + (Kd_line * D);
  lastError = error;

  setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
  return true;
}

// wall following with PID-control on distance to wall
// mode 1: left wall only (used going UP ramp)
// mode 2: both walls (unused currently)
// mode 3: right wall only (used going DOWN ramp)
bool executeWallFollow(int bSpeed, int maxPWM, int mode) {
  static float lastWallError = 0;
  static float wallIntegral = 0;
  float Kd_wall = 5.0;
  float Ki_wall = 0.3;
  float integralMax = 200.0;

  switch(mode){
    case 1:{
      int distL = getLidar(Wire, 0x10);
      if(distL < 0) distL = 999;

      if (distL < 35) {
        float wallError = wall_target - distL;
        float D = wallError - lastWallError;
        wallIntegral += wallError;
        if(wallIntegral > integralMax) wallIntegral = integralMax;
        if(wallIntegral < -integralMax) wallIntegral = -integralMax;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (Ki_wall * wallIntegral) + (Kd_wall * D);
        if(correction > bSpeed) correction = bSpeed;
        if(correction < -bSpeed) correction = -bSpeed;
        setMotors(bSpeed - correction, bSpeed + correction, maxPWM);
        return true;
      }
      lastWallError = 0;
      wallIntegral = 0;
      return false;
    }
    case 2:{
      int distL = getLidar(Wire, 0x10);
      int distR = getLidar(Wire1, 0x12);
      if(distL < 0) distL = 999;
      if(distR < 0) distR = 999;

      if (distL < 35) {
        float wallError = wall_target - distL;
        float D = wallError - lastWallError;
        wallIntegral += wallError;
        if(wallIntegral > integralMax) wallIntegral = integralMax;
        if(wallIntegral < -integralMax) wallIntegral = -integralMax;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (Ki_wall * wallIntegral) + (Kd_wall * D);
        if(correction > bSpeed) correction = bSpeed;
        if(correction < -bSpeed) correction = -bSpeed;
        setMotors(bSpeed - correction, bSpeed + correction, maxPWM);
        return true;
      } else if (distR < 35) {
        float wallError = wall_target - distR;
        float D = wallError - lastWallError;
        wallIntegral += wallError;
        if(wallIntegral > integralMax) wallIntegral = integralMax;
        if(wallIntegral < -integralMax) wallIntegral = -integralMax;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (Ki_wall * wallIntegral) + (Kd_wall * D);
        if(correction > bSpeed) correction = bSpeed;
        if(correction < -bSpeed) correction = -bSpeed;
        setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
        return true;
      }
      lastWallError = 0;
      wallIntegral = 0;
      return false;
    }
    case 3:{
      int distR = getLidar(Wire1, 0x12);
      if(distR < 0) distR = 999;

      if (distR < 35) {
        float wallError = wall_target - distR;
        float D = wallError - lastWallError;
        wallIntegral += wallError;
        if(wallIntegral > integralMax) wallIntegral = integralMax;
        if(wallIntegral < -integralMax) wallIntegral = -integralMax;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (Ki_wall * wallIntegral) + (Kd_wall * D);
        if(correction > bSpeed) correction = bSpeed;
        if(correction < -bSpeed) correction = -bSpeed;
        setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
        return true;
      }
      lastWallError = 0;
      wallIntegral = 0;
      return false;
    }
  }
  return false;
}

// simple boolean check - is there ANY line under us right now?
// used after obstacle avoidance to know when we've found the track again
bool isLineDetected() {
  uint16_t lineVals[9];
  for(int i=0; i<9; i++) { pinMode(linePins[i], OUTPUT); digitalWrite(linePins[i], HIGH); }
  delayMicroseconds(15);
  for(int i=0; i<9; i++) { pinMode(linePins[i], INPUT); lineVals[i] = 1000; }

  unsigned long st = micros();
  while(micros() - st < 1000) {
    for(int i=0; i<9; i++) {
      if(lineVals[i] == 1000 && digitalRead(linePins[i]) == LOW) lineVals[i] = micros() - st;
    }
  }

  // 500us threshold separates "seeing the line" from "seeing the floor"
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) return true;
  }
  return false;
}