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

float Kp_line = 20.0; float Kd_line = 5.0; 
float Kp_wall = 5.0; float wall_target = 130.0;
float Kp_heading = 6.0; 
int baseSpeed_6V = 380; int baseSpeed_7V = 550; 
int turning_spd = 480;
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
const unsigned long ABORT_TIME_MS = 240000; 

int base_seq = 0; 
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

bool robotEnabled() { return physical_enable && wifi_enable; }

bool isIntersection() {
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
  
  int activeCount = 0;
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) activeCount++;
  }
  return (activeCount >= 5);
}

// FIX: Added direct logging so you NEVER have to guess if the reader is working
bool readTagUID() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    stopMotors();
    strcpy(currentTag, "");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      char hex[4]; snprintf(hex, sizeof(hex), "%02X", mfrc522.uid.uidByte[i]);
      strcat(currentTag, hex);
    }
    mfrc522.PICC_HaltA();
    
    snprintf(logBuf, sizeof(logBuf), "[RFID] Scanned: %s", currentTag);
    sysLog(logBuf);
    return true;
  }
  return false;
}

void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  if (length == 0 || length == 6 || length == 21) return; 
  
  char msg[256];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  if (strstr(msg, "type=heartbeat")) {
    if (strstr(msg, "enable=1") && !wifi_enable) {
      wifi_enable = true;
      sysLog("[NET] Robot ENABLED");
    }
    else if (strstr(msg, "enable=0") && wifi_enable) {
      wifi_enable = false;
      sysLog("[NET] Robot DISABLED");
    }
  }
  
  if (strstr(msg, "type=emergency") || strstr(msg, "type=disable")) {
    wifi_enable = false;
    sysLog("[NET] KILL SWITCH TRIGGERED");
  }
  
  if (currentState == STATE_BASE_NAV && strstr(msg, "entryReply") && strstr(msg, "accepted=true")) {
    entryCleared = true;
  }
  
  if (strstr(msg, "type=openAirlockReply") && strstr(msg, "accepted=true")) {
    if (strstr(msg, "airlock=A")) airlockCleared = true;
    if (strstr(msg, "airlock=B")) airlockBCleared = true;
  }
  
  if ((currentState == STATE_WAIT_SERVER || currentState == STATE_EXIT_WAIT_SERVER) && strstr(msg, "type=isFertileReply")) {
    isFertileZone = strstr(msg, "fertile=true") != nullptr;
    waitingForServer = false;
    
    char* xLoc = strstr(msg, "x=");
    char* yLoc = strstr(msg, "y=");
    if (xLoc && yLoc) {
      currentX = atoi(xLoc + 2);
      currentY = atoi(yLoc + 2);
      snprintf(logBuf, sizeof(logBuf), "[GPS] Updated: (%d, %d)", currentX, currentY);
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

  static unsigned long lastBtn = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 300) {
    physical_enable = !physical_enable;
    lastBtn = millis();
    snprintf(logBuf, sizeof(logBuf), "[HW] Switch Toggled: %s", physical_enable ? "ARMED" : "SAFE");
    sysLog(logBuf);
  }

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

  static unsigned long lastReg = 0;
  if (millis() - lastReg > 3000) {
    char reg[64]; snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
    messenger.sendToBoard("server", reg);
    lastReg = millis();
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
  
  // FIX: Sensor hardware limit is 60Hz. Setting to 100Hz causes I2C buffer overruns.
  if (myToF.begin(0x29, Wire2)) { 
    myToF.setResolution(4 * 4); 
    myToF.setRangingFrequency(60); 
    myToF.startRanging(); 
  }
  
  for(int i=0; i<81; i++) grid[i].known = false;
  randomSeed(analogRead(0));
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
}

void checkGlobalAbort() {
  if (missionActive && millis() - missionStartTime > ABORT_TIME_MS && 
      currentState != STATE_EXIT_SEQUENCE && currentState != STATE_EXIT_DRIVE && 
      currentState != STATE_EXIT_WAIT_SERVER && currentState != STATE_AIRLOCK_WAIT_B &&
      currentState != STATE_AIRLOCK_B_DECLINE && currentState != STATE_DOCKED) {
    sysLog("[MISSION] TIMEOUT. ABORTING.");
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

  if (!robotEnabled()) { 
    stopMotors(); 
    delay(5); 
    return; 
  }

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
          if(base_seq == 0) {
            sysLog("Junc 1: Turn Left");
            turnAngle(120.0, true);
            base_seq = 1;
          } else if(base_seq == 2) {
            sysLog("Junc 1 Return: Straight");
            moveForwardTicks(400); 
            base_seq = 3;
          } else if(base_seq == 4) {
            sysLog("Junc 2 Fallback: Turn Right");
            turnAngle(120.0, false);
            base_seq = 5;
          } else {
            moveForwardTicks(500);
          }
          lostLineCount = 0;
        }
      } else {
        lostLineCount = 0;
        
        if (base_seq == 4) {
          if (isIntersection()) {
            stopMotors();
            sysLog("Junc 2 (Line Int Detected): Turn Right");
            moveForwardTicks(300); 
            turnAngle(120.0, false); 
            base_seq = 5;
          }
        }
      }

      // FIX: Decoupled Tag processing from base_seq to allow manual track testing
      if (readTagUID()) {
        baseTagCount++;
        
        if(baseTagCount == 1) {
          sysLog("[EVENT] Base Tag A Scanned");
          sysLog("[NAV] Entry Granted. 180 flip.");
          turnAngle(180.0, true);
          base_seq = 2; // Keep sequence moving forward
        } 
        else if(baseTagCount == 2) {
          sysLog("[EVENT] Base Tag B Scanned");
          char query[128]; snprintf(query, sizeof(query), "type=openAirlock airlock=A tag_id=%s board_id=%s", currentTag, BoardId);
          messenger.sendToBoard("server", query);
          airlockCleared = false;
          while(!airlockCleared) { updateUI(); delay(10); if(!robotEnabled()) return; }
          sysLog("[NAV] Airlock Open. Pushing through.");
          moveForwardTicks(800);
          base_seq = 4; // Advance to Ramp right-turn search
        }
      }

      if (base_seq == 5) {
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
          
          sysLog("[NAV] Seeking track...");
          setMotors(baseSpeed_6V, baseSpeed_6V, 440);
          unsigned long seekStart = millis();
          while(!isLineDetected() && millis() - seekStart < 4000) {
            updateUI();
            checkGlobalAbort(); if(currentState == STATE_EXIT_SEQUENCE) return;
            if(!robotEnabled()) { stopMotors(); return; }
            checkFrontObstacle();
            if(pathBlocked) {
              stopMotors();
              sysLog("[EVENT] Obstacle encountered during seek.");
              returnState = currentState;
              currentState = STATE_OBSTACLE_AVOID;
              return;
            }
            delay(5);
          }
          stopMotors();
          lostLineCount = 0;
        }
      } else lostLineCount = 0;

      if (readTagUID()) {
        bool known = false;
        for(int i=0; i<81; i++) {
          if (grid[i].known && strcmp(grid[i].uid, currentTag) == 0) { known = true; break; }
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
        char query[128]; snprintf(query, sizeof(query), "type=openAirlock airlock=B tag_id=%s board_id=%s", currentTag, BoardId);
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

      if (readTagUID()) {
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
      updateUI();
      
      if (clearance == 9999) {
        break;
      }

      if (clearance > 150) {
        setMotors(440, 440, 500); 
      } 
      else if (clearance > 35) {
        int approachSpeed = map(clearance, 35, 150, 180, 400); 
        setMotors(approachSpeed, approachSpeed, 440);
      } 
      else {
        sysLog("[RESCUE] Target Engaged. Applying pressure.");
        stopMotors();
        unsigned long waitStart = millis();
        while(millis() - waitStart < 5000) {
          updateUI();
          if(!robotEnabled()) return;
          delay(10);
        }
        
        sysLog("[RESCUE] Reversing off target.");
        setMotors(-300, -300, 440);
        waitStart = millis();
        while(millis() - waitStart < 1000) { updateUI(); if(!robotEnabled()) return; delay(1); }
        
        sysLog("[RESCUE] Seeking track...");
        while(!isLineDetected() && millis() - waitStart < 4000) {
           updateUI(); 
           if(!robotEnabled()) return; 
           delay(5); 
        }
        stopMotors();
        
        if (isLineDetected()) sysLog("[RESCUE] Track acquired.");
        else sysLog("[RESCUE] Track not found. Resuming anyway.");
        
        currentState = STATE_ARENA_NAV;
      }
      break;
    }

    case STATE_DEAD_RECKONING: 
      break;
  }
  
  delay(2); 
}