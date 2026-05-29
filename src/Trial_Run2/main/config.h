// config.h - shared definitions and externs for all modules
// we split the code into motion, nav, sensors to keep things modular
#pragma once
#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <MFRC522_I2C.h>
#include <Servo.h>
#include <MiniMessenger.h>

// finite state machine - each state maps to a phase of the challenge mission
enum RobotState {
  STATE_BASE_NAV,          // line follow from start to airlock A
  STATE_AIRLOCK_WAIT,      // (unused - kept for potential future use)
  STATE_RAMP_APPROACH,     // (unused)
  STATE_RAMP_CLIMB,        // wall-following up the ramp at higher voltage
  STATE_RAMP_DECLINE,      // wall-following down the ramp
  STATE_ARENA_NAV,         // line follow + RFID scanning in the arena
  STATE_WAIT_SERVER,       // blocked waiting for server fertility reply
  STATE_PLANT_SEED,        // deploy seed at fertile location
  STATE_OBSTACLE_AVOID,    // 3-leg bypass manoeuvre around obstacle
  STATE_REVIVE_TARGET,     // task 7/8 - approach and push stranded robot's button
  STATE_DEAD_RECKONING,    // (placeholder for future gyro-only nav)
  STATE_EXIT_SEQUENCE,     // GPS-guided routing towards airlock B position
  STATE_EXIT_DRIVE,        // driving towards next tag during exit
  STATE_EXIT_WAIT_SERVER,  // waiting for GPS update during exit
  STATE_AIRLOCK_WAIT_B,    // waiting for airlock B to open
  STATE_AIRLOCK_B_DECLINE, // descending ramp back into base
  STATE_DOCKED             // terminal state - mission complete
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

// dual-interlock safety: both must be true for the robot to move
extern volatile bool physical_enable;
extern bool wifi_enable;
extern bool pathBlocked;

extern int enc1A; extern int enc1B;
extern int enc2A; extern int enc2B;
extern volatile long pos1; extern volatile long pos2;

extern int emitterOdd; extern int emitterEven;
extern int linePins[9];
extern int weights[9];
extern char lastScannedTag[32];

// PD gains tuned empirically on the arena surface
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
bool robotEnabled();
void updateUI();
void normalizeHeading();
void sysLog(const char* message);