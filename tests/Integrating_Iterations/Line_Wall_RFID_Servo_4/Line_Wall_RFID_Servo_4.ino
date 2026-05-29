#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <MFRC522_I2C.h>
#include <Servo.h>

MotoronI2C mc;
Adafruit_MPU6050 imu;
SparkFun_VL53L5CX myToF;
MFRC522_I2C mfrc522(0x28, -1, &Wire); 
Servo seedServo;                      

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp_line = 5; 
float Kd_line = 2.5; 
float Kp_wall = 5; // Tunable constant for wall following
int baseSpeed = 400; 
float lastError = 0;
int turning_spd = 660; 

bool pathBlocked = false;
int obstacleThreshold = 200; // Dropped to 20cm to avoid reading the floor
float wall_target = 130.0;
int lostLineCount = 0;
float z_bias = 0.0;

int currentServoAngle = 0; // Tracks the 45-degree mechanical progression

// Hardware Interrupts
void tick1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++; else pos1--;
}
void tick2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++; else pos2--;
}

void setMotors(int l, int r) {
  if(l > 660) l = 660;
  if(l < -660) l = -660;
  if(r > 660) r = 660;
  if(r < -660) r = -660;
  mc.setSpeed(1, -l); 
  mc.setSpeed(3, -r); 
}

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

void turnAngle(float targetAngle, bool turnLeft) {
  setMotors(0, 0);
  delay(300); 
  
  float currentYaw = 0;
  unsigned long lastIMUTime = micros();
  
  int lSpeed = turnLeft ? -turning_spd : turning_spd;
  int rSpeed = turnLeft ? turning_spd : -turning_spd;
  
  setMotors(lSpeed, rSpeed);
  float overshoot_offset = 12.0; 
  float actualTarget = targetAngle - overshoot_offset;
  
  while(abs(currentYaw) < actualTarget) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    
    unsigned long now = micros();
    float dt = (now - lastIMUTime) / 1000000.0;
    lastIMUTime = now;
    
    float gyroZ = (g.gyro.z - z_bias) * 57.2958; 
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;
  }
  
  setMotors(0, 0);
  delay(300); 
  lastError = 0; 
}

void moveForwardTicks(long targetTicks) {
  pos1 = 0; pos2 = 0;
  setMotors(baseSpeed, baseSpeed);
  while(abs(pos1) < targetTicks && abs(pos2) < targetTicks) {
    delay(1);
  }
  setMotors(0, 0);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire1.begin(); Wire2.begin();

  pinMode(emitterOdd, OUTPUT); pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH); digitalWrite(emitterEven, HIGH);

  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  seedServo.attach(5);
  seedServo.write(currentServoAngle); // Initialize to 0

  mc.setBus(&Wire1); mc.setAddress(0x10);
  mc.reinitialize(); mc.clearResetFlag(); mc.disableCommandTimeout();
  mc.setPwmMode(1, 6); mc.setPwmMode(3, 6);

  mfrc522.PCD_Init();

  if (!imu.begin(0x68, &Wire1)) Serial.println("imu fail");
  else {
    imu.setGyroRange(MPU6050_RANGE_1000_DEG);
    Serial.println("Calibrating IMU... DO NOT TOUCH");
    float sum = 0;
    for(int i=0; i<200; i++) {
      sensors_event_t a, g, t;
      imu.getEvent(&a, &g, &t);
      sum += g.gyro.z;
      delay(5);
    }
    z_bias = sum / 200.0;
  }
  
  if (!myToF.begin(0x29, Wire2)) Serial.println("tof fail");
  else { 
    myToF.setResolution(4 * 4); 
    myToF.setRangingFrequency(15); // Dropped to 15Hz to stabilize I2C bus
    myToF.startRanging(); 
  }
  
  delay(2000); 
}

void loop() {
  // --- PRIORITY 1: FRONT COLLISION OVERRIDE ---
  if (myToF.isDataReady()) {
    VL53L5CX_ResultsData data;
    myToF.getRangingData(&data);
    int hits = 0;
    // Indices 4 to 11 cover the exact middle horizontal block of the 4x4 matrix
    for(int i = 4; i < 12; i++) {
      if(data.distance_mm[i] > 0 && data.distance_mm[i] < obstacleThreshold) hits++;
    }
    pathBlocked = (hits >= 2);
  }

  if (pathBlocked) {
    setMotors(0, 0);
    int distL = getLidar(Wire, 0x10);
    int distR = getLidar(Wire1, 0x12);
    
    if (distL > distR) turnAngle(90.0, true);
    else turnAngle(90.0, false);
    lostLineCount = 0;
    return; 
  }

  // --- PRIORITY 2: TASK EXECUTION (RFID) ---
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.print("Tag Detected:");
        Serial.print("UID: ");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      if(mfrc522.uid.uidByte[i] < 0x10) Serial.print("0"); // Leading zero
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println("\n");
    setMotors(0, 0); 
    delay(1000);
    
    moveForwardTicks(700); 
    
    mfrc522.PICC_HaltA(); 

    // Actuate seed drop by incrementing exactly 45 degrees
    currentServoAngle += 40;
    if(currentServoAngle > 180) currentServoAngle = 0; // Reset if it maxes out
    seedServo.write(currentServoAngle);
    delay(1000);

    lastError = 0; 
    return;
  }

  // --- PRIORITY 3: TRACKING (LINE OR WALL) ---
  uint16_t lineVals[9];
  for(int i=0; i<9; i++) { pinMode(linePins[i], OUTPUT); digitalWrite(linePins[i], HIGH); }
  delayMicroseconds(15);
  for(int i=0; i<9; i++) { pinMode(linePins[i], INPUT); lineVals[i] = 1000; }
  
  unsigned long st = micros();
  while(micros() - st < 1000) {
    for(int i=0; i<9; i++) {
      if(lineVals[i] == 1000 && digitalRead(linePins[i]) == LOW) {
        lineVals[i] = micros() - st;
      }
    }
  }

  long num = 0; long den = 0;
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 500) { 
      num += (long)lineVals[i] * weights[i];
      den += lineVals[i];
    }
  }

  if (den == 0) {
    lostLineCount++;
    
    if (lostLineCount > 10) {
      // Line is completely gone. Switch to Wall Following Mode.
      int distL = getLidar(Wire, 0x10);
      int distR = getLidar(Wire1, 0x12);
      
      if(distL < 0) distL = 999;
      if(distR < 0) distR = 999;

      // If left wall is within 250mm, follow it (target 130mm distance)
      if (distL < 350) {
        float wallError = wall_target - distL; 
        float correction = Kp_wall * wallError; 
        setMotors(baseSpeed - correction, baseSpeed + correction);
      } 
      // If right wall is within 250mm, follow it (target 130mm distance)
      else if (distR < 350) {
        float wallError = wall_target - distR; 
        float correction = Kp_wall * wallError;
        setMotors(baseSpeed + correction, baseSpeed - correction);
      } 
      // If NO walls are found, just stop and do the IMU turn to find a path
      else {
        setMotors(0, 0);
        delay(300);
        if (distL > distR) turnAngle(90.0, true);
        else turnAngle(90.0, false);
        lostLineCount = 0;
      }
    } else {
      // Coast to debounce the sensors
      float correction = Kp_line * lastError;
      setMotors(baseSpeed + correction, baseSpeed - correction);
    }
  } 
  else {
    // Normal Line Following PD Math
    lostLineCount = 0; 
    float error = (float)num / den;
    float P = error;
    float D = error - lastError;
    float correction = (Kp_line * P) + (Kd_line * D);
    lastError = error;

    setMotors(baseSpeed + correction, baseSpeed - correction);
  }
}