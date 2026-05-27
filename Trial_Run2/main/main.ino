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

volatile bool physical_enable = true;   
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
RobotState returnState = START_STATE; // Used to resume after avoiding obstacles

bool airlockCleared = false;
bool waitingForServer = false;
bool isFertileZone = false;
unsigned long serverWaitStartTime = 0;
unsigned long flatGroundTime = 0;
char currentTag[32] = "";

void tick1() { if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++; else pos1--; }
void tick2() { if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++; else pos2--; }

volatile unsigned long lastInterruptTime = 0;
void toggleEnableISR() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 300) {
    physical_enable = !physical_enable;
    lastInterruptTime = interruptTime;
  }
}

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
  
  // FIX: Accurate API parsing for the fertility check
  if (currentState == STATE_WAIT_SERVER && strstr(msg, "type=isFertileReply")) {
    isFertileZone = strstr(msg, "fertile=true") != nullptr;
    waitingForServer = false;
  }
}

void updateUI() {
  messenger.loop();

  static unsigned long lastBlink = 0;
  static bool ledOn = false;

  if (robotEnabled()) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, digitalRead(REVIVAL_BUTTON_PIN) == LOW ? HIGH : LOW);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW); 
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    }
  }

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
  pinMode(LED_PIN, OUTPUT); 
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggleEnableISR, FALLING);
  
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
}

void loop() {
  updateUI();

  if (!robotEnabled()) { stopMotors(); return; }

  // FIX: Trigger bypass sequence if object is detected
  if (currentState != STATE_REVIVE_TARGET && currentState != STATE_OBSTACLE_AVOID) {
    checkFrontObstacle();
    if (pathBlocked) {
      stopMotors();
      Serial.println("Obstacle Detected - Initiating Bypass Maneuver");
      returnState = currentState; // Remember what we were doing
      currentState = STATE_OBSTACLE_AVOID;
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
      executeWallFollow(baseSpeed_7V, 514, 1);
      
      // FIX: Check if we crested the hill and are now going down
      if (pitch < -5.0) {
        currentState = STATE_RAMP_DECLINE;
        flatGroundTime = 0;
      }
      else if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 2000) currentState = STATE_ARENA_NAV;
      } else flatGroundTime = 0;
      break;

    case STATE_RAMP_DECLINE:
      // Drop back to safe base speed so gravity doesn't accelerate us into a crash
      executeWallFollow(baseSpeed_6V, 440, 1); 
      if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 2000) currentState = STATE_ARENA_NAV;
      } else flatGroundTime = 0;
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
      } else lostLineCount = 0;

      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        stopMotors();
        
        strcpy(currentTag, "");
        for (byte i=0; i<mfrc522.uid.size; i++) {
          char hex[4]; snprintf(hex, sizeof(hex), "%02X", mfrc522.uid.uidByte[i]);
          strcat(currentTag, hex);
        }
        mfrc522.PICC_HaltA();
        
        char query[128]; snprintf(query, sizeof(query), "type=isFertile tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", query);
        waitingForServer = true; 
        serverWaitStartTime = millis();
        currentState = STATE_WAIT_SERVER;
      }
      break;

    case STATE_WAIT_SERVER:
      stopMotors();
      if (!waitingForServer) currentState = isFertileZone ? STATE_PLANT_SEED : STATE_ARENA_NAV;
      else if (millis() - serverWaitStartTime > 5000) currentState = STATE_ARENA_NAV;
      break;

    case STATE_PLANT_SEED:
      moveForwardTicks(640);
      currentServoAngle += 40;
      if(currentServoAngle > 180) currentServoAngle = 0;
      seedServo.write(currentServoAngle);
      
      for(int d=0; d<15; d++) { delay(100); updateUI(); }
      
      {
        char notify[128]; snprintf(notify, sizeof(notify), "type=seedPlanted tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", notify);
      }
      lastError = 0;
      currentState = STATE_ARENA_NAV;
      break;

    case STATE_OBSTACLE_AVOID: {
      // FIX: Execute Task 7 Geometric Bypass Maneuver
      turnAngle(90.0, true);           // 1. Turn Left
      moveStraightDeadReckoning(800);  // 2. Drive outward to clear object width
      turnAngle(90.0, false);          // 3. Turn Right
      moveStraightDeadReckoning(1200); // 4. Drive forward past the object length
      turnAngle(90.0, false);          // 5. Turn Right 
      
      // 6. Drive back towards the track until the line sensors see black
      setMotors(baseSpeed_6V, baseSpeed_6V, 440);
      while (!isLineDetected()) {
        updateUI();
        if(!robotEnabled()) { stopMotors(); return; }
        delay(1);
      }
      stopMotors();
      
      turnAngle(90.0, true);           // 7. Turn Left to re-align with track heading
      
      pathBlocked = false;             // Reset safety flag
      currentState = returnState;      // Resume previous FSM task
      break;
    }

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

    case STATE_DEAD_RECKONING: 
      break;
  }
}