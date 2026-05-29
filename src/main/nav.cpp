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

// wall following with P-control on distance to wall
// mode 1: left wall only (used going UP ramp)
// mode 2: both walls (unused currently)
// mode 3: right wall only (used going DOWN ramp)
// target offset is 65mm from wall - tuned so robot stays centred in tunnel
bool executeWallFollow(int bSpeed, int maxPWM, int mode) {
  switch(mode){
    case 1:{
      int distL = getLidar(Wire, 0x10);
      if(distL < 0) distL = 999;

      if (distL < 350) {
        float wallError = 65 - distL;
        setMotors(bSpeed - (Kp_wall * wallError), bSpeed + (Kp_wall * wallError), maxPWM);
        return true;
      }
      return false;
      break;
    }
    case 2:{
      int distL = getLidar(Wire, 0x10);
      int distR = getLidar(Wire1, 0x12);
      if(distL < 0) distL = 999;
      if(distR < 0) distR = 999;

      if (distL < 350) {
        float wallError = wall_target - distL;
        setMotors(bSpeed - (Kp_wall * wallError), bSpeed + (Kp_wall * wallError), maxPWM);
        return true;
      } else if (distR < 350) {
        float wallError = wall_target - distR;
        setMotors(bSpeed + (Kp_wall * wallError), bSpeed - (Kp_wall * wallError), maxPWM);
        return true;
      }
      return false;
      break;
    }
    case 3:{
      int distR = getLidar(Wire1, 0x12);
      if(distR < 0) distR = 999;

      if (distR < 350) {
        float wallError = 65 - distR;
        setMotors(bSpeed + (Kp_wall * wallError), bSpeed - (Kp_wall * wallError), maxPWM);
        return true;
      }
      return false;
      break;
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