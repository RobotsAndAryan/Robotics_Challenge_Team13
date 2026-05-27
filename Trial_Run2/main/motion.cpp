#include "config.h"
#include "motion.h"
#include <Arduino.h>

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
  
  while(abs(currentYaw) < actualTarget) {
    updateUI(); 
    if(!robotEnabled()) { stopMotors(); return; }

    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;
    float gyroZ = (g.gyro.z - z_bias) * 57.2958; 
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;
  }
  
  stopMotors();
  dStart = millis();
  while(millis() - dStart < 300) { updateUI(); if(!robotEnabled()) return; delay(1); }
  lastError = 0; 
}

void moveForwardTicks(long targetTicks) {
  pos1 = 0; pos2 = 0;
  setMotors(baseSpeed_6V, baseSpeed_6V, 440);
  // FIX: Using OR to prevent slip-induced infinite loops
  while(abs(pos1) < targetTicks || abs(pos2) < targetTicks) { 
    updateUI(); 
    if(!robotEnabled()) { stopMotors(); return; }
    delay(1); 
  }
  stopMotors();
}

void moveStraightDeadReckoning(long targetTicks) {
  pos1 = 0; pos2 = 0;
  float currentYaw = 0;
  unsigned long lastIMUTime = micros();
  
  while(abs(pos1) < targetTicks && abs(pos2) < targetTicks) {
    updateUI();
    if(!robotEnabled()) { stopMotors(); return; }

    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;
    
    float gyroZ = (g.gyro.z - z_bias) * 57.2958; 
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;

    float headingError = 0.0 - currentYaw; 
    float correction = Kp_heading * headingError;

    setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);
    delay(2); 
  }
  stopMotors();
}