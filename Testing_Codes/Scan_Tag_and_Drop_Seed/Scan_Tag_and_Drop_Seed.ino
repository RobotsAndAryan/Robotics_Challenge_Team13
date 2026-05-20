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
int linePins[] = {30, 29, 28, 27, 26, 25, 24, 23, 22};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp = 10; 
float Kd = 5.0; 
int baseSpeed = 330; 
float lastError = 0;
int turning_spd = 500; // Adjusted for 10.9V

bool pathBlocked = false;
int obstacleThreshold = 200; 
int lostLineCount = 0;
float z_bias = 0.0;

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

// Blocks main loop to execute an exact IMU turn
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

// Blocks main loop to drive a set distance using encoders
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

  seedServo.attach(32);
  seedServo.write(0);

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
    myToF.setRangingFrequency(60); 
    myToF.startRanging(); 
  }
  
  delay(2000); 
}

void loop() {
  // PRIORITY 1: FRONT COLLISION OVERRIDE
  if (myToF.isDataReady()) {
    VL53L5CX_ResultsData data;
    myToF.getRangingData(&data);
    int hits = 0;
    for(int i = 4; i < 12; i++) {
      if(data.distance_mm[i] > 0 && data.distance_mm[i] < obstacleThreshold) hits++;
    }
    pathBlocked = (hits >= 2);
  }

  if (pathBlocked) {
    setMotors(0, 0);
    return; // Hard loop restart. Ignore all other logic until clear.
  }

  // PRIORITY 2: TASK EXECUTION (RFID)
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    setMotors(0, 0); // Halt for stability
    delay(200);
    
    // Drive predefined distance (TUNE THIS TICK VALUE TO YOUR DESIRED CM)
    moveForwardTicks(150); 
    
    // Actuate seed drop mechanism
    seedServo.write(90);
    delay(500);
    seedServo.write(0);
    delay(500);
    
    mfrc522.PICC_HaltA(); // Acknowledge tag so it doesn't double-fire
    lastError = 0; // Reset tracking history
    return;
  }

  // PRIORITY 3: LINE TRACKING & GAP NAVIGATION
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
    // Increment blind counter
    lostLineCount++;
    
    // If blind for ~10 consecutive loops (approx 50-100ms), assume the grid gap
    if (lostLineCount > 10) {
      setMotors(0, 0);
      delay(300); // Let chassis settle
      
      int distL = getLidar(Wire, 0x10);
      int distR = getLidar(Wire1, 0x12);
      
      // Filter out I2C failures (-1) as zeros
      if(distL < 0) distL = 0;
      if(distR < 0) distR = 0;

      // Turn towards the side with more physical space
      if (distL > distR) {
        turnAngle(90.0, true);
      } else {
        turnAngle(90.0, false);
      }
      
      lostLineCount = 0; // Reset counter after turn
    } else {
      // Coast forward using last known error while we wait for the debounce
      float P = lastError;
      float correction = Kp * P;
      setMotors(baseSpeed + correction, baseSpeed - correction);
    }
  } 
  else {
    // Normal PD Math
    lostLineCount = 0; 
    float error = (float)num / den;
    float P = error;
    float D = error - lastError;
    float correction = (Kp * P) + (Kd * D);
    lastError = error;

    setMotors(baseSpeed + correction, baseSpeed - correction);
  }
}