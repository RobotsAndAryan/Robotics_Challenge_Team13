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
  delay(300); 
  float currentYaw = 0;
  unsigned long lastIMUTime = micros();
  int lSpeed = turnLeft ? -400 : 400;
  int rSpeed = turnLeft ? 400 : -400;
  
  setMotors(lSpeed, rSpeed, 800);
  float actualTarget = targetAngle - 12.0; 
  
  while(abs(currentYaw) < actualTarget) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;
    float gyroZ = (g.gyro.z - z_bias) * 57.2958; 
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;
  }
  stopMotors();
  delay(300); 
  lastError = 0; 
}

void moveForwardTicks(long targetTicks) {
  pos1 = 0; pos2 = 0;
  setMotors(baseSpeed_6V, baseSpeed_6V, 440);
  while(abs(pos1) < targetTicks && abs(pos2) < targetTicks) { delay(1); }
  stopMotors();
}
