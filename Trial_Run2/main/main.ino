#include "config.h"
#include "motion.h"
#include "sensors.h"
#include "nav.h"
#include "secrets.h"

// Instantiate globals
MotoronI2C mc;
Adafruit_MPU6050 imu;
SparkFun_VL53L5CX myToF;
MFRC522_I2C mfrc522(0x28, -1, &Wire); 
Servo seedServo;
MiniMessenger messenger;

const char* BoardId = "Kayubo";
const int LED_PIN = 4;
const int BUTTON_PIN = 2;
bool physical_enable = true;   
bool wifi_enable = false;      
bool pathBlocked = false;

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp_line = 10; float Kd_line = 5.0; 
float Kp_wall = 2.5; 
int baseSpeed_6V = 300; 
int baseSpeed_7V = 400; 
float lastError = 0;
int obstacleThreshold = 200; 
int lostLineCount = 0;
int currentServoAngle = 0;
float z_bias = 0.0;

enum RobotState {
  STATE_BASE_NAV,
  STATE_AIRLOCK_WAIT,
  STATE_RAMP_CLIMB,
  STATE_ARENA_GRID,
  STATE_WAIT_FERTILITY,
  STATE_PLANT_SEED
};
RobotState currentState = STATE_BASE_NAV;

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
  if (currentState == STATE_WAIT_FERTILITY && strstr(msg, "isFertileReply")) {
    isFertileZone = strstr(msg, "fertile=true") != nullptr;
    waitingForServer = false;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire1.begin(); Wire2.begin();

  pinMode(emitterOdd, OUTPUT); pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH); digitalWrite(emitterEven, HIGH);
  pinMode(LED_PIN, OUTPUT); pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  seedServo.attach(46); seedServo.write(0);

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
  messenger.loop();

  static unsigned long lastBtn = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 300) {
    physical_enable = !physical_enable;
    lastBtn = millis();
  }

  if (physical_enable) {
    static unsigned long lastReg = 0;
    if (millis() - lastReg > 10000) {
      char reg[64]; snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
      messenger.sendToBoard("server", reg);
      lastReg = millis();
    }
  }

  if (!robotEnabled()) { stopMotors(); return; }

  checkFrontObstacle();
  if (pathBlocked) { stopMotors(); return; }

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
      if (airlockCleared) { currentState = STATE_RAMP_CLIMB; flatGroundTime = 0; }
      break;

    case STATE_RAMP_CLIMB:
      executeWallFollow(baseSpeed_7V, 514); 
      if (abs(pitch) < 5.0) {
        if (flatGroundTime == 0) flatGroundTime = millis();
        else if (millis() - flatGroundTime > 2000) currentState = STATE_ARENA_GRID;
      } else flatGroundTime = 0;
      break;

    case STATE_ARENA_GRID:
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        if(++lostLineCount > 10) stopMotors(); 
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
        waitingForServer = true; serverWaitStartTime = millis();
        currentState = STATE_WAIT_FERTILITY;
      }
      break;

    case STATE_WAIT_FERTILITY:
      stopMotors();
      if (!waitingForServer) currentState = isFertileZone ? STATE_PLANT_SEED : STATE_ARENA_GRID;
      else if (millis() - serverWaitStartTime > 5000) currentState = STATE_ARENA_GRID;
      break;

    case STATE_PLANT_SEED:
      moveForwardTicks(150);
      currentServoAngle = (currentServoAngle + 45) % 180;
      seedServo.write(currentServoAngle);
      for(int d=0; d<15; d++) { delay(100); messenger.loop(); }
      char notify[128]; snprintf(notify, sizeof(notify), "type=seedPlanted tag_id=%s board_id=%s", currentTag, BoardId);
      messenger.sendToBoard("server", notify);
      lastError = 0;
      currentState = STATE_ARENA_GRID;
      break;
  }
}
