#include "config.h"
#include "nav.h"
#include "motion.h"
#include "sensors.h"
#include <Arduino.h>

bool executeLineFollow(int bSpeed, int maxPWM) {
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

  long num = 0; long den = 0;
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) {
      num += (long)lineVals[i] * weights[i];
      den += lineVals[i];
    }
  }

  if (den == 0) {
    int spinSpd = (bSpeed > 400) ? 350 : bSpeed; 
    if (lastError > 0) setMotors(spinSpd, -spinSpd, maxPWM);
    else if (lastError < 0) setMotors(-spinSpd, spinSpd, maxPWM);
    return false; 
  }

  float error = (float)num / den;
  float P = error;
  float D = error - lastError;
  float correction = (Kp_line * P) + (Kd_line * D);
  lastError = error;

  setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
  return true;
}

bool executeWallFollow(int bSpeed, int maxPWM, int mode) {
  static float lastWallError = 0;
  float kd_w = 25.0; 
  
  switch(mode){
    case 1:{
      int distL = getLidar(Wire, 0x10);
      if(distL <= 0 || distL > 900) distL = 999;

      if (distL < 350) { 
        float wallError = wall_target - distL; 
        float D = wallError - lastWallError;
        lastWallError = wallError;
        
        float correction = (Kp_wall * wallError) + (kd_w * D);
        
        if(correction > bSpeed * 0.5) correction = bSpeed * 0.5;
        if(correction < -bSpeed * 0.5) correction = -bSpeed * 0.5;
        
        // FIX: Inverted motor logic. Left track must speed up to steer Right (away from left wall).
        setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
        return true;
      } else {
        // Wall lost fallback: Drift slightly left to safely re-acquire it.
        setMotors(bSpeed - 60, bSpeed + 60, maxPWM);
        return false;
      }
    }
    case 2:{
      int distL = getLidar(Wire, 0x10);
      int distR = getLidar(Wire1, 0x12);
      if(distL <= 0 || distL > 900) distL = 999;
      if(distR <= 0 || distR > 900) distR = 999;

      if (distL < 350) {
        float wallError = wall_target - distL;
        float D = wallError - lastWallError;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (kd_w * D);
        if(correction > bSpeed * 0.5) correction = bSpeed * 0.5;
        if(correction < -bSpeed * 0.5) correction = -bSpeed * 0.5;
        setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
        return true;
      } else if (distR < 350) {
        float wallError = wall_target - distR;
        float D = wallError - lastWallError;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (kd_w * D);
        if(correction > bSpeed * 0.5) correction = bSpeed * 0.5;
        if(correction < -bSpeed * 0.5) correction = -bSpeed * 0.5;
        setMotors(bSpeed - correction, bSpeed + correction, maxPWM);
        return true;
      }
      lastWallError = 0;
      return false;
    }
    case 3:{
      int distR = getLidar(Wire1, 0x12);
      if(distR <= 0 || distR > 900) distR = 999;

      if (distR < 350) {
        float wallError = wall_target - distR;
        float D = wallError - lastWallError;
        lastWallError = wallError;
        float correction = (Kp_wall * wallError) + (kd_w * D);
        if(correction > bSpeed * 0.5) correction = bSpeed * 0.5;
        if(correction < -bSpeed * 0.5) correction = -bSpeed * 0.5;
        // FIX: Inverted motor logic. Right track must speed up to steer Left (away from right wall).
        setMotors(bSpeed - correction, bSpeed + correction, maxPWM);
        return true;
      } else {
        // Wall lost fallback: Drift slightly right to safely re-acquire it.
        setMotors(bSpeed + 60, bSpeed - 60, maxPWM);
        return false;
      }
    }
  }
  return false;
}

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

  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) return true;
  }
  return false;
}