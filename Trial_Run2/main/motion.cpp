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
  int lSpeed = turnLeft ? -turning_spd : turning_spd;
  int rSpeed = turnLeft ? turning_spd : -turning_spd;
  
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

// NEW: Task 4 Open-Field Dead Reckoning
void moveStraightDeadReckoning(long targetTicks) {
  Serial.print("Dead Reckoning Started: "); Serial.print(targetTicks); Serial.println(" ticks");
  pos1 = 0; pos2 = 0;
  float currentYaw = 0;
  unsigned long lastIMUTime = micros();
  
  while(abs(pos1) < targetTicks && abs(pos2) < targetTicks) {
    // 1. Integrate Gyro for Heading
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;
    
    float gyroZ = (g.gyro.z - z_bias) * 57.2958; 
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;

    // 2. Proportional Correction to stay on 0 degree line
    float headingError = 0.0 - currentYaw; 
    float correction = Kp_heading * headingError;

    // 3. Apply Correction to Wheels
    setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);
    delay(2); // Stability delay
  }
  stopMotors();
  Serial.println("Dead Reckoning Complete.");
}
