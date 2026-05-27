#pragma once
#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <MFRC522_I2C.h>
#include <Servo.h>
#include <MiniMessenger.h>

enum RobotState {
  STATE_BASE_NAV,
  STATE_AIRLOCK_WAIT,
  STATE_RAMP_APPROACH,
  STATE_RAMP_CLIMB,
  STATE_RAMP_DECLINE,
  STATE_ARENA_NAV,
  STATE_WAIT_SERVER,
  STATE_PLANT_SEED,
  STATE_OBSTACLE_AVOID,
  STATE_REVIVE_TARGET,
  STATE_DEAD_RECKONING,
  STATE_EXIT_SEQUENCE // NEW: End of match abort
};

#define START_STATE STATE_BASE_NAV

extern MotoronI2C mc;
extern Adafruit_MPU6050 imu;
extern SparkFun_VL53L5CX myToF;
extern MFRC522_I2C mfrc522;
extern Servo seedServo;
extern MiniMessenger messenger;

extern const char* BoardId;
extern const int LED_PIN;
extern const int BUTTON_PIN;
extern const int GREEN_LED_PIN;
extern const int REVIVAL_BUTTON_PIN;

extern volatile bool physical_enable;   
extern bool wifi_enable;      
extern bool pathBlocked;

extern int enc1A; extern int enc1B;
extern int enc2A; extern int enc2B;
extern volatile long pos1; extern volatile long pos2;

extern int emitterOdd; extern int emitterEven;
extern int linePins[9];
extern int weights[9]; 

extern float Kp_line; extern float Kd_line; 
extern float Kp_wall; extern float wall_target;
extern float Kp_heading; 
extern int baseSpeed_6V; extern int baseSpeed_7V; 
extern int turning_spd;
extern float lastError;
extern int obstacleThreshold; 
extern int lostLineCount;
extern int currentServoAngle;
extern float z_bias;

void tick1();
void tick2();
void toggleEnableISR();
bool robotEnabled();
void updateUI();