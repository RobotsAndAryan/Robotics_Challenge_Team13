#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_VL53L5CX_Library.h>

MotoronI2C mc;
Adafruit_MPU6050 imu;
SparkFun_VL53L5CX myToF;

int enc1A = 44; int enc1B = 45;
int enc2A = 39; int enc2B = 40;
volatile long pos1 = 0; volatile long pos2 = 0;

int emitterOdd = 37; int emitterEven = 38;
int linePins[] = {22,23,24,25,26,27,28,29,30};
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp = 10; 
float Kd = 7.50; 
int baseSpeed = 330; 
float lastError = 0;
unsigned long lastPrint = 0;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();
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

  if (!imu.begin(0x68, &Wire1)) Serial.println("imu fail");
  if (!myToF.begin(0x29, Wire2)) Serial.println("tof fail");
  else { myToF.setResolution(4 * 4); myToF.startRanging(); }
  
  delay(2000); 
}

void loop() {
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

  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    int distL = getLidar(Wire, 0x10);
    int distR = getLidar(Wire1, 0x12);
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    
    Serial.print("L:"); Serial.print(distL);
    Serial.print(" R:"); Serial.print(distR);
    Serial.print(" GZ:"); Serial.print(g.gyro.z);
    Serial.print(" E1:"); Serial.print(pos1);
    Serial.print(" E2:"); Serial.print(pos2);
    
    if (myToF.isDataReady()) {
      VL53L5CX_ResultsData data;
      myToF.getRangingData(&data);
      Serial.print(" TOF:"); Serial.print(data.distance_mm[5]);
    } else {
      Serial.print(" TOF:wait");
    }
    Serial.println();
  }
}