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

int enc1A = 3; int enc1B = 4;
int enc2A = 12; int enc2B = 13;
volatile long pos1 = 0;
volatile long pos2 = 0;

int emitterOdd = 37;
int emitterEven = 38;
int linePins[] = {30,29,28,27,26,25,24,23,22};

unsigned long lastPrint = 0;

void tick1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++;
  else pos1--;
}

void tick2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++;
  else pos2--;
}

int getLidar(TwoWire &w, int addr) {
  w.beginTransmission(addr);
  w.write(0x00); 
  if(w.endTransmission() != 0) return -1; 
  
  w.requestFrom(addr, 6);
  if(w.available() >= 6) {
    int dL = w.read();
    int dH = w.read();
    for(int i=0; i<4; i++) w.read(); 
    return dL + (dH << 8);
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(emitterOdd, OUTPUT);
  pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH);
  digitalWrite(emitterEven, HIGH);

  Wire.begin();   
  Wire1.begin();  
  Wire2.begin();  

  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  mc.setBus(&Wire1);
  mc.setAddress(0x10);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.setPwmMode(1, 6);
  mc.setPwmMode(3, 6);

  mfrc522.PCD_Init();

  if (!imu.begin(0x68, &Wire1)) Serial.println("imu fail");
  
  if (!myToF.begin(0x29, Wire2)) { 
    Serial.println("tof fail");
  } else {
    myToF.setResolution(4 * 4); 
    myToF.startRanging();
  }

  Serial.println("sys ready. 'f' to twitch.");
}

void loop() {
  if(Serial.available() > 0) {
    char c = Serial.read();
    if(c == 'f') {
      mc.setSpeed(1, 100);
      mc.setSpeed(3, 100);
      delay(200);
      mc.setSpeed(1, 0);
      mc.setSpeed(3, 0);
    }
  }

  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    
    int distL = getLidar(Wire, 0x10);
    int distR = getLidar(Wire1, 0x12);
    
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    
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

    Serial.print("L:"); Serial.print(distL);
    Serial.print(" R:"); Serial.print(distR);
    Serial.print(" GZ:"); Serial.print(g.gyro.z);
    Serial.print(" E1:"); Serial.print(pos1);
    Serial.print(" E2:"); Serial.print(pos2);
    
    Serial.print(" IR:[");
    for(int i=0; i<9; i++) {
      Serial.print(lineVals[i]);
      if(i<8) Serial.print(",");
    }
    Serial.print("]");

    if (myToF.isDataReady()) {
      VL53L5CX_ResultsData data;
      myToF.getRangingData(&data);
      Serial.print(" TOF:"); Serial.print(data.distance_mm[5]);
    } else {
      Serial.print(" TOF:wait");
    }
    Serial.println();
  }

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.print("UID: ");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      if(mfrc522.uid.uidByte[i] < 0x10) Serial.print("0");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    mfrc522.PICC_HaltA(); 
  }
}