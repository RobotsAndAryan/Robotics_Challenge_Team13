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

volatile bool is_turning = false;

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp_line = 30.0; float Kd_line = 5.0; 
float Kp_wall = 50.0; float wall_target = 5.4; 
float Kp_heading = 30.0;                        
int baseSpeed_6V = 440; int baseSpeed_7V = 590; 
int turning_spd = 600; 
float lastError = 0;
int obstacleThreshold = 100; 
int lostLineCount = 0;
int currentServoAngle = 0;
float z_bias = 0.0;

float dr_targetYaw = 0.0;
float dr_currentYaw = 0.0;
unsigned long dr_lastIMUTime = 0;

RobotState currentState = START_STATE;
RobotState returnState = START_STATE; 
RobotState lastLoggedState = (RobotState)-1; 

unsigned long missionStartTime = 0;
bool missionActive = false;
const unsigned long ABORT_TIME_MS = 90000; 

int base_seq = 0;
int baseTagCount = 0;
int arenaTagCount = 0;
int seedsPlanted = 0;
const int MAX_SEEDS = 5;

bool entryCleared = false;
bool airlockCleared = false;
bool airlockBCleared = false;
bool waitingForServer = false;
bool isFertileZone = false;
unsigned long serverWaitStartTime = 0;
unsigned long flatGroundTime = 0;
char currentTag[32] = "";
char lastScannedTag[32] = ""; 

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
    case STATE_OBSTACLE_STOP : return "OBSTACLE STOPPED";
    default: return "UNKNOWN";
  }
}

void sysLog(const char* message) {
  Serial.println(message);
  if (wifi_enable) {
    messenger.sendToBoard("debug_console", message);
  }
}

void executeTurn(float targetAngle, bool turnLeft) {
  is_turning = true;
  turnAngle(targetAngle, turnLeft);
  is_turning = false;
}

void tick1() { if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++; else pos1--; }
void tick2() { if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++; else pos2--; }

bool robotEnabled() { return physical_enable && wifi_enable; }

bool isRightIntersection() {
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
  
  bool rightEdge = (lineVals[0] > 500 && lineVals[1] > 500 && lineVals[2] > 500);
  bool centerActive = (lineVals[3] > 500 || lineVals[4] > 500 || lineVals[5] > 500);
  bool leftClear = (lineVals[6] < 500 && lineVals[7] < 500);
  
  return (rightEdge && centerActive && leftClear);
}

bool isLeftIntersection() {
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
  
  bool leftEdge = (lineVals[6] > 500 && lineVals[7] > 500 && lineVals[8] > 500);
  bool centerActive = (lineVals[3] > 500 || lineVals[4] > 500 || lineVals[5] > 500);
  bool rightClear = (lineVals[0] < 500 && lineVals[1] < 500);
  
  return (leftEdge && centerActive && rightClear);
}

bool isTJunction() {
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

  bool leftEdge = (lineVals[6] > 500 && lineVals[7] > 500 && lineVals[8] > 500);
  bool rightEdge = (lineVals[0] > 500 && lineVals[1] > 500 && lineVals[2] > 500);
  bool centerActive = (lineVals[3] > 500 || lineVals[4] > 500 || lineVals[5] > 500);
  
  return (leftEdge && rightEdge && centerActive);
}

bool readTagUID() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    char tempTag[32] = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      char hex[4]; snprintf(hex, sizeof(hex), "%02X", mfrc522.uid.uidByte[i]);
      strcat(tempTag, hex);
    }
    mfrc522.PICC_HaltA();

    if (strcmp(tempTag, lastScannedTag) == 0) {
      return false; 
    }
    
    stopMotors();
    strcpy(currentTag, tempTag);
    strcpy(lastScannedTag, currentTag);
    lastError = 0.0;
    lostLineCount = 0;
    
    snprintf(logBuf, sizeof(logBuf), "[RFID] %s", currentTag);
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
      sysLog("[NET] ENABLED");
    }
    else if (strstr(msg, "enable=0") && wifi_enable) {
      wifi_enable = false;
      sysLog("[NET] DISABLED");
    }
  }
  
  if (strstr(msg, "type=disable")) {
    wifi_enable = false;
    sysLog("[NET] KILL SWITCH TRIGGERED");
  }

  if (strstr(msg, "type=emergency")) {
    sysLog("[NET] EMERGENCY WARNING - RETURNING TO BASE");
    if (currentState != STATE_EXIT_SEQUENCE && currentState != STATE_EXIT_DRIVE &&
        currentState != STATE_EXIT_WAIT_SERVER && currentState != STATE_AIRLOCK_WAIT_B &&
        currentState != STATE_AIRLOCK_B_DECLINE && currentState != STATE_DOCKED) {
      lastScannedTag[0] = '\0'; 
      currentState = STATE_EXIT_SEQUENCE;
    }
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
    
    char* yLoc = strstr(msg, "x=");
    char* xLoc = strstr(msg, "y=");
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
  if (!is_turning) {
    messenger.loop();
  }

  static unsigned long lastBtn = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 300) {
    physical_enable = !physical_enable;
    lastBtn = millis();
    snprintf(logBuf, sizeof(logBuf), "[HW] Switch Toggled: %s", physical_enable ? "ARMED" : "SAFE");
    sysLog(logBuf);
  }

  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  digitalWrite(GREEN_LED_PIN, digitalRead(REVIVAL_BUTTON_PIN) == LOW ? HIGH : LOW);
  if (robotEnabled()) {
    digitalWrite(LED_PIN, HIGH);
  } else {
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
  mc.reinitialize(); mc.clearResetFlag(); 
  mc.disableCommandTimeout(); 
  mc.setPwmMode(1, 6); mc.setPwmMode(3, 6);
  mfrc522.PCD_Init();

  if (imu.begin(0x68, &Wire1)) {
    imu.setGyroRange(MPU6050_RANGE_1000_DEG);
    float sum = 0;
    for(int i=0; i<100; i++) {
      sensors_event_t a, g, t; imu.getEvent(&a, &g, &t);
      sum += g.gyro.z; delay(2);
    }
    z_bias = sum / 100.0;
  }
  
  if (myToF.begin(0x29, Wire2)) { 
    myToF.setResolution(4 * 4); 
    myToF.setRangingFrequency(60); 
    myToF.startRanging(); 
  }
  
  for(int i=0; i<81; i++) grid[i].known = false;
  randomSeed(analogRead(0));
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

  if (digitalRead(REVIVAL_BUTTON_PIN) == LOW) {
    currentState = STATE_REVIVE_TARGET;
    sysLog("[BOOT] MODE: TASK 8 (REVIVAL)");
  } else {
    currentState = STATE_BASE_NAV;
    sysLog("[BOOT] MODE: TASKS 1-6 (FULL SEQUENCE)");
  }
}

void checkGlobalAbort() {
  if (missionActive && millis() - missionStartTime > ABORT_TIME_MS &&
      currentState != STATE_EXIT_SEQUENCE && currentState != STATE_EXIT_DRIVE &&
      currentState != STATE_EXIT_WAIT_SERVER && currentState != STATE_AIRLOCK_WAIT_B &&
      currentState != STATE_AIRLOCK_B_DECLINE && currentState != STATE_DOCKED) {
    sysLog("[MISSION] TIMEOUT. ABORTING.");
    lastScannedTag[0] = '\0';
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
    snprintf(logBuf, sizeof(logBuf), "[FSM] %s", getStateName(currentState));
    sysLog(logBuf);
    lastLoggedState = currentState;
  }

  if (!robotEnabled()) { stopMotors(); delay(5); return; }

  checkGlobalAbort();

 // FIX: Explicitly separate the Stop logic (Base) from the Bypass logic (Arena)
  if (currentState == STATE_ARENA_NAV || currentState == STATE_DEAD_RECKONING) {
      checkFrontObstacle();
      if (pathBlocked) {
          stopMotors();
          sysLog("[EVENT] Obstacle Detected - Stopping till clear");
          returnState = currentState; 
          currentState = STATE_OBSTACLE_AVOID;
          return;
      }
  } 
  else {
      checkFrontObstacle();
      if (pathBlocked) {
          stopMotors();
          sysLog("[EVENT] Obstacle Detected - Engaging FSM Bypass");
          returnState = currentState; 
          currentState = STATE_OBSTACLE_STOP;
          return;
      }
  }

  float pitch = getPitch();

  switch (currentState) {
    case STATE_BASE_NAV: {
      int navSpeed = (base_seq == 1 || base_seq == 3) ? baseSpeed_6V / 1.1 : baseSpeed_6V;
      int navMax = (base_seq == 1 || base_seq == 3) ? 440 : 500;
      
      if (!executeLineFollow(navSpeed, navMax)) {
        if(++lostLineCount > 25) {
          stopMotors();
          if (base_seq == 3) {
            sysLog("[NAV] Track gap. Pushing blind toward ramp.");
            moveStraightDeadReckoning(300); 
          } else {
            sysLog("[NAV] Track gap recovery push.");
            moveForwardTicks(400); 
          }
          lostLineCount = 0;
        }
      } else {
        lostLineCount = 0;
        long currentDist = (abs(pos1) + abs(pos2)) / 2;
        
        if (base_seq == 0 && currentDist > 500 && isTJunction()) {
          stopMotors();
          sysLog("[NAV] Junc 1: Aligning & Turning Right");
          moveForwardTicks(450); 
          executeTurn(90.0, false);
          base_seq = 1;
          pos1 = 0; pos2 = 0;
        }
        else if (base_seq == 2 && currentDist > 800 && (isTJunction() || isRightIntersection())) {
          stopMotors();
          sysLog("[NAV] Junc 2: Aligning & Turning Right again");
          moveForwardTicks(450); 
          executeTurn(90.0, false); 
          base_seq = 3;
          pos1 = 0; pos2 = 0;
        }
      }

      if (base_seq == 1 && readTagUID()) {
        sysLog("[EVENT] Tag Scanned. Requesting Airlock A.");
        char query[128]; snprintf(query, sizeof(query), "type=openAirlock airlock=A tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", query);
        airlockCleared = false;
        {
          unsigned long airlockWaitStart = millis();
          while(!airlockCleared && millis() - airlockWaitStart < 5000) {
            updateUI(); delay(10);
            if(!robotEnabled()) return;
          }
        }
        sysLog("[NAV] Airlock Open. Pushing through.");
        moveForwardTicks(800);
        base_seq = 2; 
        pos1 = 0; pos2 = 0;
      }

      if (base_seq == 3) {
        if (pitch < -5.0) {
          pitchUpCount++;
          if (pitchUpCount > 5) {
            sysLog("[NAV] Ramp incline confirmed.");
            currentState = STATE_RAMP_CLIMB;
            pitchDownCount = 0;
            flatGroundTime = 0;
          }
        } else {
          pitchUpCount = 0;
        }
      }
      break;
    }

    case STATE_RAMP_CLIMB:
          if (!executeWallFollow(baseSpeed_7V, 800, 1)) {
            setMotors(baseSpeed_7V, baseSpeed_7V, 800); 
          }
          if (abs(pitch) < 5.0) {
            if (flatGroundTime == 0) flatGroundTime = millis();
            else if (millis() - flatGroundTime > 100) {
              sysLog("[NAV] Ramp cleared. Entering Arena.");
              currentState = STATE_ARENA_NAV;
            }
          } else flatGroundTime = 0;
      break;

    case STATE_RAMP_DECLINE:
      if (!executeWallFollow(baseSpeed_6V/1.2, 440, 3)) {
        setMotors(baseSpeed_6V/1.2, baseSpeed_6V/1.2, 440);
      }
      if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 400) {
          currentState = STATE_ARENA_NAV;
        }
      } else flatGroundTime = 0;
      break;

    case STATE_ARENA_NAV: {
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 15) {
          stopMotors();
          sysLog("[NAV] Line Lost. Transitioning to FSM Dead Reckoning.");
          dr_targetYaw = globalHeading; 
          dr_currentYaw = globalHeading;
          dr_lastIMUTime = micros();
          currentState = STATE_DEAD_RECKONING;
          lostLineCount = 0;
        }
      } else lostLineCount = 0;

      if (readTagUID()) {
        arenaTagCount++;

        if (arenaTagCount == 1) {
          currentX = 9; currentY = 3;
          snprintf(logBuf, sizeof(logBuf), "[GPS] Entry Node Seeded at (%d, %d)", currentX, currentY);
          sysLog(logBuf);
        }

        waitingForServer = true;
        serverWaitStartTime = millis();
        currentState = STATE_WAIT_SERVER;

        char query[128]; snprintf(query, sizeof(query), "type=isFertile tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", query);

        if (arenaTagCount == 3) {
            sysLog("[NAV] Task 3: Turn Right at Node 3.");
            executeTurn(90.0, false);
            globalHeading -= 90.0;
            normalizeHeading();
        } else if (arenaTagCount == 4) {
            sysLog("[NAV] Task 3: Turn Left at Node 4.");
            executeTurn(90.0, true);
            globalHeading += 90.0;
            normalizeHeading();
        }
      }
      break;
    }

    case STATE_DEAD_RECKONING: {
      sensors_event_t a, g, t;
      imu.getEvent(&a, &g, &t);
      unsigned long now = micros();
      if(dr_lastIMUTime == 0) dr_lastIMUTime = now;
      float dt = (now - dr_lastIMUTime) / 1000000.0;
      dr_lastIMUTime = now;

      float gyroZ = (g.gyro.z - z_bias) * 57.2958;
      if(abs(gyroZ) > 1.0) dr_currentYaw -= gyroZ * dt; 

      float headingError = dr_targetYaw - dr_currentYaw;
      float correction = Kp_heading * headingError;

      setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);

      if (isLineDetected()) {
        stopMotors();
        sysLog("[NAV] Track re-acquired.");
        dr_lastIMUTime = 0;
        currentState = STATE_ARENA_NAV;
      }
      else if (readTagUID()) {
        stopMotors();
        dr_lastIMUTime = 0;
        arenaTagCount++;
        waitingForServer = true;
        serverWaitStartTime = millis();
        currentState = STATE_WAIT_SERVER;
        
        char query[128]; snprintf(query, sizeof(query), "type=isFertile tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", query);

        if (arenaTagCount == 3) {
            sysLog("[NAV] Task 4: Turn Right at Node 3.");
            executeTurn(90.0, false);
            globalHeading -= 90.0;
            normalizeHeading();
        } else if (arenaTagCount == 4) {
            sysLog("[NAV] Task 4: Turn Left at Node 4.");
            executeTurn(90.0, true);
            globalHeading += 90.0;
            normalizeHeading();
        }
      }
      break;
    }

    case STATE_WAIT_SERVER:
      stopMotors();
      if (!waitingForServer) {
        if (isFertileZone && seedsPlanted < MAX_SEEDS) {
          currentState = STATE_PLANT_SEED;
        } else {
          if (isFertileZone && seedsPlanted >= MAX_SEEDS) sysLog("[WARN] Seeds exhausted. Skipping plant.");
          if (!isLineDetected()) {
            dr_targetYaw = globalHeading;
            dr_currentYaw = globalHeading;
            dr_lastIMUTime = micros();
            currentState = STATE_DEAD_RECKONING;
          } else {
            currentState = STATE_ARENA_NAV;
          }
        }
      } else if (millis() - serverWaitStartTime > 5000) {
        sysLog("[ERROR] Server Timeout.");
        if (!isLineDetected()) {
            dr_targetYaw = globalHeading;
            dr_currentYaw = globalHeading;
            dr_lastIMUTime = micros();
            currentState = STATE_DEAD_RECKONING;
        } else {
            currentState = STATE_ARENA_NAV;
        }
      }
      break;

    case STATE_PLANT_SEED:
      snprintf(logBuf, sizeof(logBuf), "[ACTION] Planting Seed %d/%d.", seedsPlanted + 1, MAX_SEEDS);
      sysLog(logBuf);
      moveForwardTicks(640);
      stopMotors();
      delay(500);
      {
        int targetAngle = currentServoAngle + 45;
        if(targetAngle > 180) targetAngle = 180;
        while(currentServoAngle < targetAngle) {
          currentServoAngle += 2;
          if(currentServoAngle > targetAngle) currentServoAngle = targetAngle;
          seedServo.write(currentServoAngle);
          delay(30);
        }
      }
      for(int d=0; d<10; d++) { delay(100); updateUI(); }
      seedsPlanted++;

      {
        char notify[128]; snprintf(notify, sizeof(notify), "type=seedPlanted tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", notify);
      }
      lastError = 0;
      if (!isLineDetected()) {
          dr_targetYaw = globalHeading;
          dr_currentYaw = globalHeading;
          dr_lastIMUTime = micros();
          currentState = STATE_DEAD_RECKONING;
      } else {
          currentState = STATE_ARENA_NAV;
      }
      break;

    case STATE_EXIT_SEQUENCE: {
      if(currentX == 9 && currentY == 7) {
        stopMotors();
        sysLog("[EXIT] Target 9,7 Achieved. Requesting Airlock B.");
        char query[128]; snprintf(query, sizeof(query), "type=openAirlock airlock=B tag_id=%s board_id=%s", currentTag, BoardId);
        messenger.sendToBoard("server", query);
        currentState = STATE_AIRLOCK_WAIT_B;
        break;
      }
      if(currentX == -1 || currentY == -1) {
        sysLog("[EXIT] GPS Unknown. Driving blindly to find a node.");
        currentState = STATE_EXIT_DRIVE;
        break;
      }

      normalizeHeading();
      float desiredHeading = globalHeading;

      if(currentX > 9) desiredHeading = 90.0;       
      else if(currentX < 9) desiredHeading = 270.0; 
      else if(currentY > 7) desiredHeading = 180.0; 
      else if(currentY < 7) desiredHeading = 0.0;   

      float diff = desiredHeading - globalHeading;
      if(diff > 180.0) diff -= 360.0;
      if(diff < -180.0) diff += 360.0;

      if(abs(diff) > 10.0) {
        snprintf(logBuf, sizeof(logBuf), "[EXIT] Routing. Turn %d degrees.", (int)diff);
        sysLog(logBuf);
        if(diff > 0) executeTurn(abs(diff), true);
        else executeTurn(abs(diff), false);
        globalHeading = desiredHeading;
        normalizeHeading();
      }
      currentState = STATE_EXIT_DRIVE;
      break;
    }

    case STATE_EXIT_DRIVE: {
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 15) {
          dr_targetYaw = globalHeading;
          dr_currentYaw = globalHeading;
          dr_lastIMUTime = micros();
          currentState = STATE_DEAD_RECKONING; 
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

    case STATE_AIRLOCK_WAIT_B: {
      stopMotors();
      static unsigned long airlockBWaitStart = 0;
      if (airlockBWaitStart == 0) airlockBWaitStart = millis();

      if (airlockBCleared || millis() - airlockBWaitStart > 15000) {
        if (!airlockBCleared) sysLog("[WARN] Airlock B timeout. Proceeding.");
        sysLog("[NAV] Airlock B clear. Pushing in.");
        unsigned long time = millis();
        while(millis() - time < 8000) {
          executeLineFollow(baseSpeed_6V, 440);
          updateUI();
          if(!robotEnabled()) break;
        }
        airlockBWaitStart = 0;
        currentState = STATE_AIRLOCK_B_DECLINE;
        flatGroundTime = 0;
      }
      break;
    }

    case STATE_AIRLOCK_B_DECLINE:
      if (!executeWallFollow(baseSpeed_6V, 440, 3)) {
        setMotors(baseSpeed_6V, baseSpeed_6V, 440);
      }
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
        sysLog("[MISSION COMPLETE] Docked safely.");
        flatGroundTime = 9999; 
      }
      break;

    case STATE_OBSTACLE_STOP: {
          stopMotors(); // Ensure motors do not creep
          checkFrontObstacle();
          
          if (!pathBlocked) {
            sysLog("[EVENT] Path clear. Resuming navigation.");
            currentState = returnState;
          }
          
          break; // FIX: Break MUST be outside the if-statement to prevent FSM fall-through!
      }

    case STATE_OBSTACLE_AVOID: {
      static int avoid_seq = 0;
      static bool bypassLeft = false;
      static long totalOutwardDistance = 0;
      static unsigned long stateTimer = 0;
      static int parallelPhase = 0;
      static long clearTicks = 0;
      static float savedHeading = 0.0;
      
      static float bypass_targetYaw = 0.0;
      static float bypass_currentYaw = 0.0;
      static unsigned long bypass_lastIMUTime = 0;

      switch(avoid_seq) {
        case 0: // Phase 0: Verify & Reverse
          savedHeading = globalHeading;
          totalOutwardDistance = 0;
          
          checkFrontObstacle();
          if(!pathBlocked) {
             currentState = returnState;
             break;
          }
          
          setMotors(-300, -300, 440);
          stateTimer = millis();
          avoid_seq = 1;
          break;
          
        case 1: // Phase 1: Wait for reverse to finish, pick direction
          if (millis() - stateTimer > 600) {
            stopMotors();
            int distL = getLidar(Wire, 0x10);
            int distR = getLidar(Wire1, 0x12);
            if(distL < 0) distL = 0; 
            if(distR < 0) distR = 0;
            bypassLeft = (distL >= distR); 
            snprintf(logBuf, sizeof(logBuf), "[AVOID] L:%dcm R:%dcm. Bypassing %s.", distL, distR, bypassLeft ? "LEFT" : "RIGHT");
            sysLog(logBuf);
            avoid_seq = 2;
          }
          break;
          
        case 2: // Phase 2: Turn Outward
          executeTurn(90.0, bypassLeft);
          globalHeading += bypassLeft ? 90.0 : -90.0;
          normalizeHeading();
          pos1 = 0; pos2 = 0;
          bypass_targetYaw = globalHeading;
          bypass_currentYaw = globalHeading;
          bypass_lastIMUTime = micros();
          setMotors(baseSpeed_6V, baseSpeed_6V, 440);
          parallelPhase = 0;
          avoid_seq = 3;
          break;
          
        case 3: { // Phase 3: Drive Outward (Dynamic Width)
          long currentOutward = (abs(pos1) + abs(pos2)) / 2;
          int distSide = bypassLeft ? getLidar(Wire1, 0x12) : getLidar(Wire, 0x10);
          
          if (parallelPhase == 0) {
              if (distSide > 0 && distSide <= 40) {
                  parallelPhase = 1; 
              } else if (currentOutward > 500) {
                  parallelPhase = 1; // Failsafe
              }
          } else if (parallelPhase == 1) {
              if (distSide > 40 || distSide <= 0) { 
                  parallelPhase = 2;
                  clearTicks = currentOutward;
              }
          } else if (parallelPhase == 2) {
              if (currentOutward > (clearTicks + 450)) {
                  totalOutwardDistance += currentOutward;
                  stopMotors();
                  stateTimer = millis();
                  avoid_seq = 4;
                  break;
              }
          }
          
          checkFrontObstacle();
          if (pathBlocked) {
            totalOutwardDistance += currentOutward;
            stopMotors();
            sysLog("[AVOID] Trapped moving outward. Aborting bypass.");
            globalHeading = savedHeading;
            normalizeHeading();
            pathBlocked = false;
            currentState = returnState;
            avoid_seq = 0;
            break;
          }
          
          sensors_event_t a, g, t; imu.getEvent(&a, &g, &t);
          unsigned long now = micros(); float dt = (now - bypass_lastIMUTime) / 1000000.0; bypass_lastIMUTime = now;
          float gyroZ = (g.gyro.z - z_bias) * 57.2958;
          if(abs(gyroZ) > 1.0) bypass_currentYaw -= gyroZ * dt;
          float correction = Kp_heading * (bypass_targetYaw - bypass_currentYaw);
          setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);
          break;
        }
        
        case 4: // Phase 4: Settle & Turn Parallel
          if (millis() - stateTimer > 300) {
            sysLog("[AVOID] Outward cleared. Turning parallel.");
            executeTurn(90.0, !bypassLeft);
            globalHeading += !bypassLeft ? 90.0 : -90.0;
            normalizeHeading();
            pos1 = 0; pos2 = 0;
            bypass_targetYaw = globalHeading;
            bypass_currentYaw = globalHeading;
            bypass_lastIMUTime = micros();
            parallelPhase = 0;
            setMotors(baseSpeed_6V, baseSpeed_6V, 440);
            avoid_seq = 5;
          }
          break;
          
        case 5: { // Phase 5: Drive Parallel (Dynamic Length & Chaining)
          long currentParallel = (abs(pos1) + abs(pos2)) / 2;
          
          checkFrontObstacle();
          if (pathBlocked) {
             sysLog("[AVOID] Secondary obstacle ahead! Chaining bypass wider.");
             stopMotors();
             avoid_seq = 2; // Chain loop back to turn outward
             break;
          }

          int distSide = bypassLeft ? getLidar(Wire1, 0x12) : getLidar(Wire, 0x10);
          
          if (parallelPhase == 0) {
              if (distSide > 0 && distSide <= 40) { 
                  parallelPhase = 1; 
                  sysLog("[AVOID] Parallel: Obstacle side detected.");
              } else if (currentParallel > 800) {
                  parallelPhase = 1; 
              }
          } 
          else if (parallelPhase == 1) {
              if (distSide > 40 || distSide <= 0) { 
                  parallelPhase = 2; 
                  clearTicks = currentParallel;
                  sysLog("[AVOID] Parallel: Sensor cleared obstacle. Pushing tail clearance.");
              }
          }
          else if (parallelPhase == 2) {
              if (currentParallel > (clearTicks + 600)) { 
                  sysLog("[AVOID] Parallel: Tail clearance achieved.");
                  stopMotors();
                  stateTimer = millis();
                  avoid_seq = 6;
                  break; 
              }
          }

          sensors_event_t a, g, t; imu.getEvent(&a, &g, &t);
          unsigned long now = micros(); float dt = (now - bypass_lastIMUTime) / 1000000.0; bypass_lastIMUTime = now;
          float gyroZ = (g.gyro.z - z_bias) * 57.2958;
          if(abs(gyroZ) > 1.0) bypass_currentYaw -= gyroZ * dt;
          float correction = Kp_heading * (bypass_targetYaw - bypass_currentYaw);
          setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);
          break;
        }
        
        case 6: // Phase 6: Settle & Turn Inward
          if (millis() - stateTimer > 300) {
            sysLog("[AVOID] Returning to track axis.");
            executeTurn(90.0, !bypassLeft);
            globalHeading += !bypassLeft ? 90.0 : -90.0;
            normalizeHeading();
            pos1 = 0; pos2 = 0;
            bypass_targetYaw = globalHeading;
            bypass_currentYaw = globalHeading;
            bypass_lastIMUTime = micros();
            setMotors(baseSpeed_6V, baseSpeed_6V, 440);
            avoid_seq = 7;
          }
          break;
          
        case 7: { // Phase 7: Drive Inward
          long currentInward = (abs(pos1) + abs(pos2)) / 2;
          
          if (isLineDetected()) {
             sysLog("[AVOID] Track detected.");
             stopMotors();
             avoid_seq = 8;
             break;
          }
          if (currentInward > (totalOutwardDistance + 300)) {
             sysLog("[AVOID] Reached target lateral displacement. Stopping search.");
             stopMotors();
             avoid_seq = 8;
             break;
          }
          
          checkFrontObstacle();
          if (pathBlocked) {
             sysLog("[AVOID] Blocked while returning! Forcing realignment.");
             stopMotors();
             avoid_seq = 8;
             break;
          }

          sensors_event_t a, g, t; imu.getEvent(&a, &g, &t);
          unsigned long now = micros(); float dt = (now - bypass_lastIMUTime) / 1000000.0; bypass_lastIMUTime = now;
          float gyroZ = (g.gyro.z - z_bias) * 57.2958;
          if(abs(gyroZ) > 1.0) bypass_currentYaw -= gyroZ * dt;
          float correction = Kp_heading * (bypass_targetYaw - bypass_currentYaw);
          setMotors(baseSpeed_6V - correction, baseSpeed_6V + correction, 440);
          break;
        }
        
        case 8: // Phase 8: Final Alignment
          sysLog("[AVOID] Restoring original heading.");
          executeTurn(90.0, bypassLeft); 
          globalHeading = savedHeading; 
          normalizeHeading();          
          pathBlocked = false;             
          currentState = returnState;
          avoid_seq = 0; // Reset for next obstacle
          break;
      }
      break; 
    }

    case STATE_REVIVE_TARGET: {
      static unsigned long reviveStartTime = 0;
      if (reviveStartTime == 0) reviveStartTime = millis();

      if (millis() - reviveStartTime > 30000) {
        sysLog("[RESCUE] Approach timeout. Aborting.");
        stopMotors();
        reviveStartTime = 0;
        currentState = STATE_ARENA_NAV;
        break;
      }

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
        setMotors(150, 150, 440);
        unsigned long waitStart = millis();
        while(millis() - waitStart < 3000) {
          updateUI();
          if(!robotEnabled()) { stopMotors(); return; }
          delay(10);
        }
        stopMotors();

        sysLog("[RESCUE] Reversing off target.");
        setMotors(-300, -300, 440);
        waitStart = millis();
        while(millis() - waitStart < 1200) { updateUI(); if(!robotEnabled()) { stopMotors(); return; } delay(1); }
        stopMotors();

        sysLog("[RESCUE] Seeking track...");
        setMotors(200, 200, 440);
        unsigned long seekStart = millis();
        while(!isLineDetected() && millis() - seekStart < 4000) {
           updateUI();
           if(!robotEnabled()) { stopMotors(); return; }
           delay(5);
        }
        stopMotors();

        if (isLineDetected()) sysLog("[RESCUE] Track acquired.");
        else sysLog("[RESCUE] Track not found. Resuming anyway.");

        reviveStartTime = 0;
        currentState = STATE_ARENA_NAV;
      }
      break;
    }

    default:
      stopMotors();
      break;
  }
  delay(2); 
}