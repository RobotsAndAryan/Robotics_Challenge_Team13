#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <MFRC522_I2C.h>
#include <Servo.h>
#include <MiniMessenger.h>
#include "secrets.h" // Ensure your WiFi credentials are here

// --- Core Objects ---
MotoronI2C mc;
Adafruit_MPU6050 imu;
SparkFun_VL53L5CX myToF;
MFRC522_I2C mfrc522(0x28, -1, &Wire); 
Servo seedServo;
MiniMessenger messenger;

// --- Network & UI ---
const char* BoardId = "Kayubo";
const int LED_PIN = 4;
const int BUTTON_PIN = 2;
bool physical_enable = true;   
bool wifi_enable = false;      

// --- Hardware Pins ---
int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

// --- Control Variables ---
float Kp_line = 10; 
float Kd_line = 5.0; 
float Kp_wall = 2.5; 
int baseSpeed_6V = 300; 
int baseSpeed_7V = 400; // Increased for ramp
float lastError = 0;
bool pathBlocked = false;
int obstacleThreshold = 200; 
int lostLineCount = 0;
int currentServoAngle = 0;

// --- FSM States ---
enum RobotState {
  STATE_BASE_NAV,
  STATE_RAMP_CLIMB,
  STATE_ARENA_NAV,
  STATE_WAIT_SERVER,
  STATE_PLANT_SEED
};
RobotState currentState = STATE_BASE_NAV;

// --- Async Server Flags ---
bool waitingForServer = false;
bool isFertileZone = false;
unsigned long serverWaitStartTime = 0;

// --- Hardware Interrupts ---
void tick1() { if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++; else pos1--; }
void tick2() { if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++; else pos2--; }

// --- Safe Motor Actuation ---
void setMotors(int l, int r, int max_pwm) {
  if(l > max_pwm) l = max_pwm;
  if(l < -max_pwm) l = -max_pwm;
  if(r > max_pwm) r = max_pwm;
  if(r < -max_pwm) r = -max_pwm;
  mc.setSpeed(1, -l); 
  mc.setSpeed(3, -r); 
}

void stopMotors() { setMotors(0, 0, 800); }

bool robotEnabled() { return physical_enable && wifi_enable; }

// --- Server Callback ---
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  if (!physical_enable || length == 0) return;

  char msg[256];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.print("NET IN: "); Serial.println(msg);

  if (strstr(msg, "type=heartbeat")) {
    if (strstr(msg, "enable=1")) wifi_enable = true;
    else if (strstr(msg, "enable=0")) wifi_enable = false;
  }

  if (strstr(msg, "type=disable enabled=false reason=operator")) {
    wifi_enable = false;
  }

  // Handle Fertility Response
  if (waitingForServer && strstr(msg, "type=fertility_resp")) {
    if (strstr(msg, "fertile=1")) isFertileZone = true;
    else isFertileZone = false;
    waitingForServer = false; // Unblock the FSM
  }
}

// --- Sensor Helpers ---
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

// --- FSM Actions ---
bool executeLineFollow(int bSpeed, int maxPWM) {
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

  long num = 0; long den = 0;
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) { 
      num += (long)lineVals[i] * weights[i];
      den += lineVals[i];
    }
  }

  if (den == 0) return false; // Line lost

  float error = (float)num / den;
  float P = error;
  float D = error - lastError;
  float correction = (Kp_line * P) + (Kd_line * D);
  lastError = error;

  setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
  return true;
}

void executeWallFollow(int bSpeed, int maxPWM) {
  int distL = getLidar(Wire, 0x10);
  int distR = getLidar(Wire1, 0x12);
  
  if(distL < 0) distL = 999;
  if(distR < 0) distR = 999;

  if (distL < 250) {
    float wallError = 100.0 - distL; 
    float correction = Kp_wall * wallError; 
    setMotors(bSpeed - correction, bSpeed + correction, maxPWM);
  } else if (distR < 250) {
    float wallError = 100.0 - distR; 
    float correction = Kp_wall * wallError;
    setMotors(bSpeed + correction, bSpeed - correction, maxPWM);
  } else {
    setMotors(bSpeed, bSpeed, maxPWM); // Coast if completely blind
  }
}

// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire1.begin(); Wire2.begin();

  pinMode(emitterOdd, OUTPUT); pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH); digitalWrite(emitterEven, HIGH);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  seedServo.attach(46); seedServo.write(currentServoAngle);

  mc.setBus(&Wire1); mc.setAddress(0x10);
  mc.reinitialize(); mc.clearResetFlag(); mc.disableCommandTimeout();
  mc.setPwmMode(1, 6); mc.setPwmMode(3, 6);

  mfrc522.PCD_Init();
  if (!imu.begin(0x68, &Wire1)) Serial.println("imu fail");
  if (!myToF.begin(0x29, Wire2)) Serial.println("tof fail");
  else { myToF.setResolution(4 * 4); myToF.setRangingFrequency(15); myToF.startRanging(); }

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("System Boot Complete.");
}

// --- Main Loop ---
void loop() {
  messenger.loop();

  // 1. Hardware UI Toggle
  static unsigned long lastPress = 0;
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW && millis() - lastPress >= 300) {
    lastPress = millis();
    physical_enable = !physical_enable;
  }
  lastButtonState = currentButtonState;

  // 2. Network Registration (Increased Frequency to 3 seconds)
  if (physical_enable) {
    static unsigned long lastRegister = 0;
    if (millis() - lastRegister >= 3000) {
      lastRegister = millis();
      char reg[64];
      snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
      Serial.println("Registration Sent");
      messenger.sendToBoard("server", reg);
    }
  }

  // 3. LED Status
  static unsigned long lastBlink = 0;
  static bool ledOn = false;
  if (!robotEnabled()) {
    stopMotors();
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    }
    return; // HALT FSM IF DISABLED
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // 4. Front Collision Safety Override
  if (myToF.isDataReady()) {
    VL53L5CX_ResultsData data;
    myToF.getRangingData(&data);
    int hits = 0;
    for(int i = 4; i < 12; i++) {
      if(data.distance_mm[i] > 0 && data.distance_mm[i] < obstacleThreshold) hits++;
    }
    if (hits >= 2) {
      stopMotors();
      return; // Suspend FSM until clear
    }
  }

  // 5. Read IMU Pitch (Assuming X-axis is Forward)
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958;

  // 6. FINITE STATE MACHINE
  switch (currentState) {

    case STATE_BASE_NAV:
      executeLineFollow(baseSpeed_6V, 440);
      
      // Check for Ramp Transition (> 12 degrees incline)
      if (pitch > 12.0) {
        Serial.println("Ramp Detected. Switching to STATE_RAMP_CLIMB");
        currentState = STATE_RAMP_CLIMB;
      }
      // Check for RFID
      else if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        mfrc522.PICC_HaltA();
        Serial.println("Tag Detected. Switching to STATE_ARENA_NAV");
        // Usually base nav to arena boundary tag.
        currentState = STATE_ARENA_NAV; 
      }
      break;

    case STATE_RAMP_CLIMB:
      // Wall follow with overvolted max clamp (514 = ~7V)
      executeWallFollow(baseSpeed_7V, 514);
      
      // Check if leveled out (< 5 degrees)
      if (pitch < 5.0) {
        Serial.println("Level ground. Switching to STATE_ARENA_NAV");
        currentState = STATE_ARENA_NAV;
      }
      break;

    case STATE_ARENA_NAV:
      if (!executeLineFollow(baseSpeed_6V, 440)) {
        lostLineCount++;
        if(lostLineCount > 10) executeWallFollow(baseSpeed_6V, 440);
      } else {
        lostLineCount = 0;
      }

      // Read Tags in Arena
      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        stopMotors();
        
        // Construct UID String
        char uidStr[32] = "";
        for (byte i = 0; i < mfrc522.uid.size; i++) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X", mfrc522.uid.uidByte[i]);
          strcat(uidStr, hex);
        }
        mfrc522.PICC_HaltA();

        // Query Server
        char query[128];
        snprintf(query, sizeof(query), "type=rfid_check uid=%s", uidStr);
        messenger.sendToBoard("server", query);
        
        waitingForServer = true;
        serverWaitStartTime = millis();
        currentState = STATE_WAIT_SERVER;
        Serial.println("Sent to server. Waiting for fertility status...");
      }
      break;

    case STATE_WAIT_SERVER:
      stopMotors();
      // We are waiting for onMessage() to flip the waitingForServer boolean
      if (!waitingForServer) {
        if (isFertileZone) {
          Serial.println("Zone is Fertile. Planting Seed.");
          currentState = STATE_PLANT_SEED;
        } else {
          Serial.println("Zone is Infertile. Moving on.");
          currentState = STATE_ARENA_NAV;
        }
      }
      // Timeout safety (if server drops connection)
      else if (millis() - serverWaitStartTime > 5000) {
        waitingForServer = false;
        Serial.println("Server Timeout. Ignoring tag.");
        currentState = STATE_ARENA_NAV;
      }
      break;

    case STATE_PLANT_SEED:
      // 1. Move Forward Slightly (Non-blocking tick check using a quick while loop)
      pos1 = 0; pos2 = 0;
      setMotors(baseSpeed_6V, baseSpeed_6V, 440);
      while(abs(pos1) < 200 && abs(pos2) < 200) { delay(1); } // Tune 200
      stopMotors();
      delay(500);

      // 2. Drop Seed (Exactly 45 deg)
      currentServoAngle += 45;
      if(currentServoAngle > 180) currentServoAngle = 0;
      seedServo.write(currentServoAngle);
      
      // 3. We use a blocking delay here intentionally to let mechanics finish 
      // before sensors kick back in, but we still call messenger.loop() so we don't disconnect.
      for(int d=0; d<15; d++) { delay(100); messenger.loop(); }

      lastError = 0; // Clean slate
      currentState = STATE_ARENA_NAV;
      break;
  }
}