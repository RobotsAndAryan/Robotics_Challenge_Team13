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

volatile bool physical_enable = false;   
bool wifi_enable = false;      
bool pathBlocked = false;

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp_line = 20.0; float Kd_line = 5.0; 
float Kp_wall = 5.0; float wall_target = 130.0;
float Kp_heading = 6.0; 
int baseSpeed_6V = 350; int baseSpeed_7V = 550; 
int turning_spd = 400;
float lastError = 0;
int obstacleThreshold = 100; 
int lostLineCount = 0;
int currentServoAngle = 0;
float z_bias = 0.0;

RobotState currentState = START_STATE;
RobotState returnState = START_STATE; 
RobotState lastLoggedState = (RobotState)-1; 

unsigned long missionStartTime = 0;
bool missionActive = false;
const unsigned long ABORT_TIME_MS = 300000; 

int baseTagCount = 0;
bool entryCleared = false;
bool airlockCleared = false;
bool airlockBCleared = false;
bool waitingForServer = false;
bool isFertileZone = false;
unsigned long serverWaitStartTime = 0;
unsigned long flatGroundTime = 0;
char currentTag[32] = "";

struct ArenaNode {
  char uid[32];
  int x;
  int y;
  bool known;
};
ArenaNode grid[81];
int currentX = -1;
int currentY = -1;
float globalHeading = 0.0; 

int pitchUpCount = 0;
int pitchDownCount = 0;

char logBuf[128]; 

const char* getStateName(RobotState state) {
  switch(state) {
    case STATE_BASE_NAV: return "BASE_NAV";
    case STATE_AIRLOCK_WAIT: return "AIRLOCK_WAIT";
    case STATE_RAMP_APPROACH: return "RAMP_APPROACH";
    case STATE_RAMP_CLIMB: return "RAMP_CLIMB";
    case STATE_RAMP_DECLINE: return "RAMP_DECLINE";
    case STATE_ARENA_NAV: return "ARENA_NAV";
    case STATE_WAIT_SERVER: return "WAIT_SERVER";
    case STATE_PLANT_SEED: return "PLANT_SEED";
    case STATE_OBSTACLE_AVOID: return "OBSTACLE_AVOID";
    case STATE_REVIVE_TARGET: return "REVIVE_TARGET";
    case STATE_DEAD_RECKONING: return "DEAD_RECKONING";
    case STATE_EXIT_SEQUENCE: return "EXIT_SEQUENCE";
    case STATE_EXIT_DRIVE: return "EXIT_DRIVE";
    case STATE_EXIT_WAIT_SERVER: return "EXIT_WAIT_SERVER";
    case STATE_AIRLOCK_WAIT_B: return "AIRLOCK_WAIT_B";
    case STATE_AIRLOCK_B_DECLINE: return "AIRLOCK_B_DECLINE";
    case STATE_DOCKED: return "DOCKED";
    default: return "UNKNOWN";
  }
}

void sysLog(const char* message) {
  Serial.println(message);
  if (wifi_enable) {
    messenger.sendToBoard("debug_console", message);
  }
}

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
    if (strstr(msg, "enable=1") && !wifi_enable) {
      wifi_enable = true;
      sysLog("[NET] Robot ENABLED by Server");
    }
    else if (strstr(msg, "enable=0") && wifi_enable) {
      wifi_enable = false;
      sysLog("[NET] Robot DISABLED by Server");
    }
  }
  if (strstr(msg, "type=emergency") || strstr(msg, "type=disable")) {
    wifi_enable = false;
    sysLog("[NET] KILL SWITCH TRIGGERED");
  }
  
  if (currentState == STATE_BASE_NAV && strstr(msg, "entryReply") && strstr(msg, "accepted=true")) {
    entryCleared = true;
  }
  if (currentState == STATE_AIRLOCK_WAIT && strstr(msg, "openAirlockReply") && strstr(msg, "accepted=true")) {
    airlockCleared = true;
  }
  if (currentState == STATE_AIRLOCK_WAIT_B && strstr(msg, "openAirlockBReply") && strstr(msg, "accepted=true")) {
    airlockBCleared = true;
  }
  
  if ((currentState == STATE_WAIT_SERVER || currentState == STATE_EXIT_WAIT_SERVER) && strstr(msg, "type=isFertileReply")) {
    isFertileZone = strstr(msg, "fertile=true") != nullptr;
    waitingForServer = false;
    
    char* xLoc = strstr(msg, "x=");
    char* yLoc = strstr(msg, "y=");
    if (xLoc && yLoc) {
      currentX = atoi(xLoc + 2);
      currentY = atoi(yLoc + 2);
      snprintf(logBuf, sizeof(logBuf), "[GPS] Location Updated: (%d, %d)", currentX, currentY);
      sysLog(logBuf);
      
      for(int i=0; i<81; i++) {
        if(!grid[i].known) {
          strcpy(grid[i].uid, currentTag);
          grid[i].x = currentX;
          grid[i].y = currentY;
          grid[i].known = true;
          break;
        } else if (strcmp(grid[i].uid, currentTag) == 0) {
          break;
        }
      }
    }
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

void normalizeHeading() {
  while(globalHeading >= 360.0) globalHeading -= 360.0;
  while(globalHeading < 0.0) globalHeading += 360.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire1.begin(); Wire2.begin();

  sysLog("\n[BOOT] Initializing Hardware...");

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
    sysLog("[BOOT] IMU Calibrated.");
  }
  
  if (myToF.begin(0x29, Wire2)) { 
    myToF.setResolution(4 * 4); 
    myToF.setRangingFrequency(15); 
    myToF.startRanging(); 
    sysLog("[BOOT] ToF Init OK");
  }
  
  for(int i=0; i<81; i++) grid[i].known = false;
  randomSeed(analogRead(0));
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  sysLog("[BOOT] Waiting for switch.");
}

void checkGlobalAbort() {
  if (missionActive && millis() - missionStartTime > ABORT_TIME_MS && 
      currentState != STATE_EXIT_SEQUENCE && currentState != STATE_EXIT_DRIVE && 
      currentState != STATE_EXIT_WAIT_SERVER && currentState != STATE_AIRLOCK_WAIT_B &&
      currentState != STATE_AIRLOCK_B_DECLINE && currentState != STATE_DOCKED) {
    sysLog("[MISSION] 5 MIN TIMEOUT. ABORTING.");
    currentState = STATE_EXIT_SEQUENCE;
  }
}

void loop() {
  updateUI();

  if (robotEnabled() && !missionActive) {
    missionStartTime = millis();
    missionActive = true;
    sysLog("[MISSION] Timer Started.");
  }

  if (currentState != lastLoggedState) {
    snprintf(logBuf, sizeof(logBuf), "[FSM] %s -> %s", getStateName(lastLoggedState), getStateName(currentState));
    sysLog(logBuf);
    lastLoggedState = currentState;
  }

  if (!robotEnabled()) { stopMotors(); return; }

  checkGlobalAbort();

  if (currentState != STATE_REVIVE_TARGET && currentState != STATE_OBSTACLE_AVOID && 
      currentState != STATE_EXIT_SEQUENCE && currentState != STATE_DOCKED) {
    checkFrontObstacle();
    if (pathBlocked) {
      stopMotors();
      sysLog("[EVENT] Obstacle - Bypass");
      returnState = currentState; 
      currentState = STATE_OBSTACLE_AVOID;
      return;
    }
  }

  float pitch = getPitch();

  switch (currentState) {
    case STATE_BASE_NAV:
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 8) {
          stopMotors();
          turnAngle(90.0, true);
          if(isLineDetected()) { lostLineCount=0; break; }
          
          turnAngle(90.0, false);
          moveForwardTicks(250);
          if(isLineDetected()) { lostLineCount=0; break; }
          
          setMotors(-300, -300, 440);
          unsigned long bt = millis();
          while(millis()-bt < 400) { updateUI(); delay(1); }
          stopMotors();
          
          turnAngle(90.0, false);
          if(isLineDetected()) { lostLineCount=0; break; }
          
          turnAngle(90.0, false); 
          lostLineCount = 0;
        }
      } else {
        lostLineCount = 0;
      }

      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial() && baseTagCount < 2) {
        stopMotors();
        baseTagCount++;
        mfrc522.PICC_HaltA();
        
        if(baseTagCount == 1) {
          char query[64]; snprintf(query, sizeof(query), "type=requestEntry board_id=%s", BoardId);
          messenger.sendToBoard("server", query);
          entryCleared = false;
          while(!entryCleared) { updateUI(); delay(10); if(!robotEnabled()) return; }
        } else if(baseTagCount == 2) {
          char query[64]; snprintf(query, sizeof(query), "type=openAirlockA board_id=%s", BoardId);
          messenger.sendToBoard("server", query);
          currentState = STATE_AIRLOCK_WAIT;
        }
      }
      break;

    case STATE_AIRLOCK_WAIT:
      stopMotors();
      if (airlockCleared) { 
        unsigned long time = millis();
        while(millis() - time < 10000) {
          executeLineFollow(baseSpeed_6V, 440);
          updateUI();
          if(!robotEnabled()) break;
        }
        moveForwardTicks(800);
        stopMotors();
        currentState = STATE_RAMP_APPROACH; 
        pitchUpCount = 0;
      }
      break;

    case STATE_RAMP_APPROACH:
      setMotors(440, 440, 440); 
      
      if (pitch < -10.0) {
        pitchUpCount++;
        if (pitchUpCount > 20) {
          currentState = STATE_RAMP_CLIMB;
          pitchDownCount = 0;
          flatGroundTime = 0;
        }
      } else {
        pitchUpCount = 0;
      }
      break;

    case STATE_RAMP_CLIMB:
      executeWallFollow(baseSpeed_7V, 514, 1);
      
      if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 3000) {
          currentState = STATE_ARENA_NAV;
        }
      } else flatGroundTime = 0;
      break;

    case STATE_RAMP_DECLINE:
      executeWallFollow(baseSpeed_6V, 440, 3); 
      if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 1500) {
          currentState = STATE_ARENA_NAV;
        }
      } else flatGroundTime = 0;
      break;

    case STATE_ARENA_NAV: {
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 10) {
          stopMotors();
          int r = random(0, 3);
          if(r==0) { turnAngle(90.0, true); globalHeading += 90.0; }
          else if(r==1) { turnAngle(90.0, false); globalHeading -= 90.0; }
          else moveForwardTicks(400);
          normalizeHeading();
          lostLineCount = 0;
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

        bool known = false;
        for(int i=0; i<81; i++) {
          if (grid[i].known && strcmp(grid[i].uid, currentTag) == 0) {
            known = true;
            break;
          }
        }
        
        char query[128]; snprintf(query, sizeof(query), "type=isFertile tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", query);
        
        waitingForServer = true; 
        serverWaitStartTime = millis();
        currentState = STATE_WAIT_SERVER;
      }
      break;
    }

    case STATE_WAIT_SERVER:
      stopMotors();
      if (!waitingForServer) currentState = isFertileZone ? STATE_PLANT_SEED : STATE_ARENA_NAV;
      else if (millis() - serverWaitStartTime > 5000) {
        sysLog("[ERROR] Server Timeout");
        currentState = STATE_ARENA_NAV;
      }
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

    case STATE_EXIT_SEQUENCE: {
      if(currentX == 3 && currentY == 1) {
        stopMotors();
        char query[64]; snprintf(query, sizeof(query), "type=openAirlockB board_id=%s", BoardId);
        messenger.sendToBoard("server", query);
        currentState = STATE_AIRLOCK_WAIT_B;
        break;
      }

      if(currentX == -1 || currentY == -1) {
        currentState = STATE_EXIT_DRIVE;
        break;
      }

      normalizeHeading();
      float desiredHeading = globalHeading;
      
      if(currentX > 3) desiredHeading = 90.0;       
      else if(currentX < 3) desiredHeading = 270.0; 
      else if(currentY > 1) desiredHeading = 180.0; 
      else if(currentY < 1) desiredHeading = 0.0;   

      float diff = desiredHeading - globalHeading;
      if(diff > 180.0) diff -= 360.0;
      if(diff < -180.0) diff += 360.0;

      if(abs(diff) > 10.0) {
        if(diff > 0) turnAngle(abs(diff), true);
        else turnAngle(abs(diff), false);
        globalHeading = desiredHeading;
        normalizeHeading();
      }

      currentState = STATE_EXIT_DRIVE;
      break;
    }

    case STATE_EXIT_DRIVE: {
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 10) {
          moveStraightDeadReckoning(400); 
          lostLineCount = 0;
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
        currentState = STATE_EXIT_WAIT_SERVER;
      }
      break;
    }

    case STATE_EXIT_WAIT_SERVER:
      stopMotors();
      if (!waitingForServer) currentState = STATE_EXIT_SEQUENCE;
      else if (millis() - serverWaitStartTime > 5000) currentState = STATE_EXIT_SEQUENCE;
      break;

    case STATE_AIRLOCK_WAIT_B:
      stopMotors();
      if (airlockBCleared) { 
        unsigned long time = millis();
        while(millis() - time < 8000) {
          executeLineFollow(baseSpeed_6V, 440);
          updateUI();
          if(!robotEnabled()) break;
        }
        currentState = STATE_AIRLOCK_B_DECLINE;
        flatGroundTime = 0;
      }
      break;

    case STATE_AIRLOCK_B_DECLINE:
      executeWallFollow(baseSpeed_6V, 440, 3);
      if (pitch < -8.0) {
        flatGroundTime = 0;
      } else if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 2000) {
          currentState = STATE_DOCKED;
        }
      } else flatGroundTime = 0;
      break;

    case STATE_DOCKED:
      stopMotors();
      if (flatGroundTime != 9999) {
        sysLog("[MISSION COMPLETE] Docked.");
        flatGroundTime = 9999; 
      }
      break;

    case STATE_OBSTACLE_AVOID: {
      setMotors(-300, -300, 440);
      unsigned long bTime = millis();
      while(millis() - bTime < 600) { updateUI(); delay(1); }
      stopMotors();

      turnAngle(90.0, true);  
      globalHeading += 90.0;
      normalizeHeading();         
      
      setMotors(baseSpeed_6V, baseSpeed_6V, 440);
      unsigned long bypassStart = millis();
      bool wallTrap = false;
      
      while(true) {
        updateUI();
        checkGlobalAbort(); if(currentState == STATE_EXIT_SEQUENCE) return;
        if(!robotEnabled()) { stopMotors(); return; }

        checkFrontObstacle();
        if(pathBlocked) {
          stopMotors();
          setMotors(-300, -300, 440);
          bTime = millis();
          while(millis() - bTime < 600) { updateUI(); delay(1); }
          stopMotors();
          turnAngle(90.0, true);
          globalHeading += 90.0;
          normalizeHeading();
          setMotors(baseSpeed_6V, baseSpeed_6V, 440);
          bypassStart = millis(); 
        }

        int distR = getLidar(Wire1, 0x12);
        if(distR > 0 && distR > 400 && (millis() - bypassStart > 800)) break;
        if (millis() - bypassStart > 4000) { wallTrap = true; break; }
        delay(5);
      }
      
      if (wallTrap) {
        stopMotors();
        turnAngle(180.0, true);
        globalHeading += 180.0;
        normalizeHeading();
        setMotors(baseSpeed_6V, baseSpeed_6V, 440);
        while (!isLineDetected()) {
          updateUI();
          checkGlobalAbort(); if(currentState == STATE_EXIT_SEQUENCE) return;
          if(!robotEnabled()) return; 
          delay(5); 
        }
        stopMotors();
        turnAngle(90.0, true);
        globalHeading += 90.0;
        normalizeHeading();
        pathBlocked = false;
        currentState = returnState;
        break;
      }
      
      bTime = millis();
      while(millis() - bTime < 800) { updateUI(); delay(1); }
      stopMotors();

      turnAngle(90.0, false);   
      globalHeading -= 90.0;
      normalizeHeading();       

      setMotors(baseSpeed_6V, baseSpeed_6V, 440);
      bypassStart = millis();
      wallTrap = false;
      
      while(true) {
        updateUI();
        checkGlobalAbort(); if(currentState == STATE_EXIT_SEQUENCE) return;
        if(!robotEnabled()) { stopMotors(); return; }

        checkFrontObstacle();
        if(pathBlocked) {
          stopMotors();
          setMotors(-300, -300, 440);
          bTime = millis();
          while(millis() - bTime < 600) { updateUI(); delay(1); }
          stopMotors();
          turnAngle(90.0, true);
          globalHeading += 90.0;
          normalizeHeading();
          setMotors(baseSpeed_6V, baseSpeed_6V, 440);
          bypassStart = millis();
        }

        int distR = getLidar(Wire1, 0x12);
        if(distR > 0 && distR > 400 && (millis() - bypassStart > 800)) break;
        if (millis() - bypassStart > 4000) { wallTrap = true; break; }
        delay(5);
      }
      
      if (wallTrap) {
        stopMotors();
        turnAngle(180.0, true);
        globalHeading += 180.0;
        normalizeHeading();
        setMotors(baseSpeed_6V, baseSpeed_6V, 440);
        while (!isLineDetected()) { 
          updateUI(); 
          checkGlobalAbort(); if(currentState == STATE_EXIT_SEQUENCE) return;
          if(!robotEnabled()) return; 
          delay(5); 
        }
        stopMotors();
        turnAngle(90.0, true);
        globalHeading += 90.0;
        normalizeHeading();
        pathBlocked = false;
        currentState = returnState;
        break;
      }
      
      bTime = millis();
      while(millis() - bTime < 800) { updateUI(); delay(1); }
      stopMotors();

      turnAngle(90.0, false);   
      globalHeading -= 90.0;
      normalizeHeading();       
      
      setMotors(baseSpeed_6V, baseSpeed_6V, 440);
      while (!isLineDetected()) {
        updateUI();
        checkGlobalAbort(); if(currentState == STATE_EXIT_SEQUENCE) return;
        if(!robotEnabled()) { stopMotors(); return; }

        checkFrontObstacle();
        if(pathBlocked) {
          stopMotors();
          setMotors(-300, -300, 440);
          bTime = millis();
          while(millis() - bTime < 600) { updateUI(); delay(1); }
          stopMotors();
          turnAngle(90.0, true);
          globalHeading += 90.0;
          normalizeHeading();
          setMotors(baseSpeed_6V, baseSpeed_6V, 440);
        }
        delay(5);
      }
      stopMotors();
      
      turnAngle(90.0, true); 
      globalHeading += 90.0;
      normalizeHeading();          
      pathBlocked = false;             
      currentState = returnState;      
      break;
    }

    case STATE_REVIVE_TARGET: {
      int clearance = getFrontClearanceMM();
      
      if (clearance > 800) {
        setMotors(400, 400, 500); 
      } 
      else if (clearance > 35) {
        int approachSpeed = map(clearance, 35, 800, 330, 440); 
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