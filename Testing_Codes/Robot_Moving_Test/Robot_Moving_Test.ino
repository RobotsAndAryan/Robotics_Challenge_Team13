#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MFRC522_I2C.h>
#include <SparkFun_VL53L5CX_Library.h>

MotoronI2C mc;
Adafruit_MPU6050 imu;
MFRC522_I2C mfrc522(0x28, -1, &Wire); 
SparkFun_VL53L5CX myToF;

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp = 1.5; 
float Kd = 5.0; 
int baseSpeed = 600;
float lastError = 0;
int turning_spd = 700;

unsigned long lastPrint = 0;
bool lineFollowActive = true;

float z_bias = 0.0;

void tick1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++; else pos1--;
}

void tick2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++; else pos2--;
}

void setMotors(int l, int r) {
  if(l > 800) l = 800;
  if(l < -800) l = -800;
  if(r > 800) r = 800;
  if(r < -800) r = -800;
  
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
  lineFollowActive = false;
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
    
    if(abs(gyroZ) > 1.0) { 
      currentYaw += gyroZ * dt;
    }
  }
  
  setMotors(0, 0);
  delay(300); 
  lastError = 0; 
  lineFollowActive = true;
}

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  Wire.begin();
  Wire2.begin();

  pinMode(emitterOdd, OUTPUT); pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH); digitalWrite(emitterEven, HIGH);

  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  mc.setBus(&Wire1); mc.setAddress(0x10);
  mc.reinitialize(); mc.clearResetFlag(); mc.disableCommandTimeout();
  mc.setPwmMode(1, 6); mc.setPwmMode(3, 6);

  mfrc522.PCD_Init();
  
  if (!imu.begin(0x68, &Wire1)) {
    Serial.println("imu fail");
  } else {
    imu.setGyroRange(MPU6050_RANGE_1000_DEG);
    
    Serial.println("calibrating imu... DO NOT MOVE ROBOT");
    float sum = 0;
    for(int i=0; i<200; i++) {
      sensors_event_t a, g, t;
      imu.getEvent(&a, &g, &t);
      sum += g.gyro.z;
      delay(5);
    }
    z_bias = sum / 200.0;
    Serial.print("bias: "); Serial.println(z_bias);
  }

  if (!myToF.begin(0x29, Wire2)) Serial.println("tof fail");
  else { myToF.setResolution(4 * 4); myToF.startRanging(); }
}

void loop() {
  if(Serial.available() > 0) {
    char c = Serial.read();
    if(c == 'd') turnAngle(90.0, true);  
    if(c == 'a') turnAngle(90.0, false); 
  }

  if (lineFollowActive) {
    uint16_t lineVals[9];
    for(int i=0; i<9; i++) {
      pinMode(linePins[i], OUTPUT);
      digitalWrite(linePins[i], HIGH);
    }
    delayMicroseconds(15);
    for(int i=0; i<9; i++) {
      pinMode(linePins[i], INPUT);
      lineVals[i] = 1000; 
    }
    
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
      if(lineVals[i] > 400) { 
        num += (long)lineVals[i] * weights[i];
        den += lineVals[i];
      }
    }

    float error = 0;
    if(den != 0) error = (float)num / den;
    else error = lastError;

    float P = error;
    float D = error - lastError;
    float correction = (Kp * P) + (Kd * D);
    lastError = error;

    int leftMotor = baseSpeed + correction;
    int rightMotor = baseSpeed - correction;
    setMotors(leftMotor, rightMotor);
  }

  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    int distL = getLidar(Wire, 0x10);
    int distR = getLidar(Wire1, 0x12);
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    
    Serial.print("L:"); Serial.print(distL);
    Serial.print(" R:"); Serial.print(distR);
    Serial.print(" GZ:"); Serial.println(g.gyro.z - z_bias);
  }

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    mfrc522.PICC_HaltA(); 
  }
}