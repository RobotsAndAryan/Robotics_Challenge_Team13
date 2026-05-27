#include "config.h"
#include "motion.h"
#include "sensors.h"
#include "nav.h"
#include "secrets.h"

MotoronI2C mc;
Adafruit_MPU6050 imu;
SparkFun_VL53L5CX myToF;
MFRC522_I2C mfrc522(0x28, -1, &Wire); 
Servo seedServo;
MiniMessenger messenger;

const char* BoardId = "Kayubo";
const int LED_PIN = 4;
const int BUTTON_PIN = 2;
const int GREEN_LED_PIN = 3;
const int REVIVAL_BUTTON_PIN = 46;

bool physical_enable = true;   
bool wifi_enable = false;      
bool pathBlocked = false;

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp_line = 10.0; float Kd_line = 5.0; 
float Kp_wall = 5.0; float wall_target = 130.0;
float Kp_heading = 6.0; 
int baseSpeed_6V = 440; int baseSpeed_7V = 550; 
int turning_spd = 550;
float lastError = 0;
int obstacleThreshold = 200; 
int lostLineCount = 0;
int currentServoAngle = 0;
float z_bias = 0.0;

RobotState currentState = START_STATE;

bool airlockCleared = false;
bool waitingForServer = false;
bool isFertileZone = false;
unsigned long serverWaitStartTime = 0;
unsigned long flatGroundTime = 0;
char currentTag[32] = "";

void tick1() { if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++; else pos1--; }
void tick2() { if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++; else pos2--; }
bool robotEnabled() { return physical_enable && wifi_enable; }

void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  if (!physical_enable || length == 0) return;
  char msg[256];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  if (strstr(msg, "type=heartbeat")) {
    if (strstr(msg, "enable=1")) wifi_enable = true;
    else if (strstr(msg, "enable=0")) wifi_enable = false;
  }
  if (strstr(msg, "type=emergency") || strstr(msg, "type=disable")) wifi_enable = false;
  if (currentState == STATE_AIRLOCK_WAIT && strstr(msg, "openAirlockReply") && strstr(msg, "accepted=true")) airlockCleared = true;
  if (currentState == STATE_WAIT_SERVER && strstr(msg, "fertility_resp")) {
    isFertileZone = strstr(msg, "fertile=1") != nullptr;
    waitingForServer = false;
  }
}

// FIX: Centralized UI Polling. This runs everywhere, even inside motion loops.
void updateUI() {
  messenger.loop();

  // 1. Hardware Kill Switch Toggle
  static unsigned long lastBtn = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 300) {
    physical_enable = !physical_enable;
    lastBtn = millis();
    Serial.print("Physical Enable Toggled: "); Serial.println(physical_enable);
  }

  // 2. LED Status Management (Mutually Exclusive)
  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  if (robotEnabled()) {
    digitalWrite(LED_PIN, HIGH); // Solid Red
    // Only allow Green LED to light up if the robot is actually active
    digitalWrite(GREEN_LED_PIN, digitalRead(REVIVAL_BUTTON_PIN) == LOW ? HIGH : LOW);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW); // Force off
    if (millis() - lastBlink >= 500) { // Blink Red
      lastBlink = millis();
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    }
  }

  // 3. Network Heartbeat
  if (physical_enable) {
    static unsigned long lastReg = 0;
    if (millis() - lastReg > 5000) {
      char reg[64]; snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
      messenger.sendToBoard("server", reg);
      lastReg = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire1.begin(); Wire2.begin();

  pinMode(emitterOdd, OUTPUT); pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH); digitalWrite(emitterEven, HIGH);
  pinMode(LED_PIN, OUTPUT); pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_LED_PIN, OUTPUT); digitalWrite(GREEN_LED_PIN, LOW);
  pinMode(REVIVAL_BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  seedServo.attach(5); seedServo.write(0);

  mc.setBus(&Wire1); mc.setAddress(0x10);
  mc.reinitialize(); mc.clearResetFlag(); mc.disableCommandTimeout();
  mc.setPwmMode(1, 6); mc.setPwmMode(3, 6);
  mfrc522.PCD_Init();

  if (imu.begin(0x68, &Wire1)) {
    imu.setGyroRange(MPU6050_RANGE_1000_DEG);
    float sum = 0;
    for(int i=0; i<200; i++) {
      sensors_event_t a, g, t; imu.getEvent(&a, &g, &t);
      sum += g.gyro.z; delay(5);
    }
    z_bias = sum / 200.0;
  }
  
  if (myToF.begin(0x29, Wire2)) { 
    myToF.setResolution(4 * 4); 
    myToF.setRangingFrequency(15); 
    myToF.startRanging(); 
  }

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.print("Booted into Mode: "); Serial.println(currentState);
}

void loop() {
  updateUI();

  if (!robotEnabled()) { stopMotors(); return; }

  // Bypass front obstacle check ONLY if we are doing the revival rescue sequence
  if (currentState != STATE_REVIVE_TARGET) {
    checkFrontObstacle();
    if (pathBlocked) {
      Serial.println("Obstacle Detected - Emergency Halt");
      stopMotors();
      return;
    }
  }

  float pitch = getPitch();

  switch (currentState) {
    case STATE_BASE_NAV:
      executeLineFollow(baseSpeed_6V, 440);
      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        stopMotors();
        char query[64]; snprintf(query, sizeof(query), "type=openAirlockA board_id=%s", BoardId);
        messenger.sendToBoard("server", query);
        mfrc522.PICC_HaltA();
        currentState = STATE_AIRLOCK_WAIT;
      }
      break;

    case STATE_AIRLOCK_WAIT:
      stopMotors();
      if (airlockCleared) { 
        currentState = STATE_RAMP_CLIMB; 
        flatGroundTime = 0; 
      }
      break;

    case STATE_RAMP_CLIMB:
      moveStraightDeadReckoning(1200);
      executeWallFollow(baseSpeed_7V, 514, 1);
      if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 2000) {
          currentState = STATE_ARENA_NAV;
        }
      } else {
        flatGroundTime = 0;
      }
      break;

    case STATE_ARENA_NAV:
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 10) {
          if(!executeWallFollow(baseSpeed_6V, 440, 2)) {
            moveStraightDeadReckoning(800); 
            
            stopMotors();
            unsigned long delayStart = millis();
            while(millis() - delayStart < 300) { updateUI(); if(!robotEnabled()) return; delay(1); }
            
            int distL = getLidar(Wire, 0x10);
            int distR = getLidar(Wire1, 0x12);
            if (distL > distR) turnAngle(90.0, true);
            else turnAngle(90.0, false);
            lostLineCount = 0;
          }
        }
      } else {
        lostLineCount = 0;
      }

      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        stopMotors();
        unsigned long delayStart = millis();
        while(millis() - delayStart < 1000) { updateUI(); if(!robotEnabled()) return; delay(1); }
        
        moveForwardTicks(640);
        mfrc522.PICC_HaltA();
        
        currentServoAngle += 40;
        if(currentServoAngle > 180) currentServoAngle = 0;
        seedServo.write(currentServoAngle);
        
        delayStart = millis();
        while(millis() - delayStart < 1000) { updateUI(); if(!robotEnabled()) return; delay(1); }
        lastError = 0;
      }
      break;

    case STATE_REVIVE_TARGET: {
      int clearance = getFrontClearanceMM();
      
      if (clearance > 800) {
        setMotors(440, 440, 500); 
      } 
      else if (clearance > 35) {
        int approachSpeed = map(clearance, 35, 800, 400, 514); 
        setMotors(approachSpeed, approachSpeed, 440);
      } 
      else {
        stopMotors();
        // FIX: Active polling delay replaces static delay(5000)
        unsigned long waitStart = millis();
        while(millis() - waitStart < 2000) {
          updateUI();
          if(!robotEnabled()) return;
          delay(10);
        }
        
        setMotors(-300, -300, 440);
        waitStart = millis();
        while(millis() - waitStart < 1000) { updateUI(); if(!robotEnabled()) return; delay(1); }
        stopMotors();
        
        currentState = STATE_ARENA_NAV;
      }
      break;
    }

    case STATE_WAIT_SERVER:
    case STATE_PLANT_SEED:
    case STATE_DEAD_RECKONING: 
      break;
  }
}
