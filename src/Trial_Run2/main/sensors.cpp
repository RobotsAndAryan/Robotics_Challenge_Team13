// sensors.cpp - abstracts all sensor reads so other modules don't deal with I2C directly
#include "config.h"
#include "sensors.h"
#include <Arduino.h>

// reads distance from VL53L0X lidar over I2C - returns mm or -1 on failure
// the sensor sends 6 bytes: 2 for distance (little-endian), 4 status bytes we discard
int getLidar(TwoWire &w, int addr) {
  w.beginTransmission(addr);
  w.write(0x00);
  if(w.endTransmission() != 0) return -1;
  w.requestFrom(addr, 6);
  if(w.available() >= 6) {
    int dL = w.read(); int dH = w.read();
    for(int i=0; i<4; i++) w.read();
    return dL + (dH << 8);
  }
  return -1;
}

// uses the 4x4 ToF depth array - pixels 4-11 cover the front-facing region
// we require 2+ pixels below threshold to avoid false triggers from noise
void checkFrontObstacle() {
  if (myToF.isDataReady()) {
    VL53L5CX_ResultsData data;
    myToF.getRangingData(&data);
    int hits = 0;
    for(int i = 4; i < 12; i++) {
      if(data.distance_mm[i] > 0 && data.distance_mm[i] < obstacleThreshold) hits++;
    }
    pathBlocked = (hits >= 2);
  }
}

// averages the front 8 pixels for a smooth distance estimate (used in rescue mode)
int getFrontClearanceMM() {
  if (myToF.isDataReady()) {
    VL53L5CX_ResultsData data;
    myToF.getRangingData(&data);
    int sum = 0; int valid = 0;
    for(int i = 4; i < 12; i++) {
      if(data.distance_mm[i] > 0) {
        sum += data.distance_mm[i];
        valid++;
      }
    }
    if (valid > 0) return sum / valid;
  }
  return 9999;
}

// pitch from accelerometer using atan2 - gives angle of incline/decline
// used to detect when we're on the ramp vs flat ground
float getPitch() {
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  return atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958;
}