/*Wiring Architecture : 
 Reflectance : D22-D30, 3V3 GND
 RFID: SDA(20) SCL(21) [Wire], 5V GND
 Left Lidar (0x12): SDA(20) SCL(21) [Wire], 5V GND, Pin 5 to GND
 Right Lidar (0x13): SDA1 SCL1 [Wire1], 5V GND, Pin 5 to GND
 IMU (0x68): SDA1 SCL1 [Wire1], 5V GND
 Motoron (0x10): SDA1 SCL1 [Wire1] stacked
 64 ToF Imager: SDA2 SCL2 [Wire2], 3V3 GND
 Motors: VMOT to battery. Encoders to D3, D4 (Left) and D12, D13 (Right), 5V GND
 Buttons: D2 (Kill), D36 (Spare)
 RGB LED: D35 (Red), D33 (Green)
 Servo: D32, 5V GND (FROM BATTERY NOT ARDUINO)
*/

#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MFRC522_I2C.h>
#include <Servo.h>

MotoronI2C mc;
Adafruit_MPU6050 imu;
MFRC522_I2C mfrc522(0x28, -1, &Wire); 
Servo myServo;

int killBtn = 2; 
int spareBtn = 36;
int ledRed = 35;
int ledGreen = 33;
int servoPin = 32;

int enc1A = 3; int enc1B = 4;
int enc2A = 12; int enc2B = 13;

int linePins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};

volatile long pos1 = 0;
volatile long pos2 = 0;

int servoPos[] = {0, 45, 90, 135, 180};
int currentServoIdx = 0;

bool isStopped = true;
unsigned long lastBlink = 0;
bool ledState = false;

float yaw = 0;
unsigned long lastImuTime = 0;
int lastBtn = LOW;

void tick1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++;
  else pos1--;
}

void tick2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++;
  else pos2--;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(killBtn, INPUT);
  pinMode(spareBtn, INPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  
  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

  for(int i=0; i<9; i++) pinMode(linePins[i], INPUT);

  myServo.attach(servoPin);
  myServo.write(0);

  Wire.begin();   // I2C0
  Wire1.begin();  // I2C1
  Wire2.begin();  // I2C2

  mfrc522.PCD_Init();

  mc.setBus(&Wire1);
  mc.setAddress(0x10);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.setMaxDeceleration(1, 400);
  mc.setMaxDeceleration(2, 400);
  mc.setMaxAcceleration(1, 400);
  mc.setMaxAcceleration(2, 400);
  mc.setPwmMode(1, 6);
  mc.setPwmMode(2, 6);

  if (!imu.begin(0x68, &Wire1)) {
    Serial.println("imu dead on wire1");
  }

  Serial.println("grading demo ready");
  Serial.println("1=fwd slow, 2=fwd fast, 3=L90, 4=R90, 5=U-Turn");
  Serial.println("l=lines, d=lidars, r=rfid, e=encoders, p=servo");
  lastImuTime = millis();
}

void updateIMU() {
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  float dt = (millis() - lastImuTime) / 1000.0;
  lastImuTime = millis();
  
  if(abs(g.gyro.z) > 0.05) {
    yaw += g.gyro.z * 57.2958 * dt; 
  }
}

int getLidar(TwoWire &w, int addr) {
  w.beginTransmission(addr);
  w.write(0x01);
  w.write(0x02);
  w.write(7);
  w.endTransmission();
  delay(10);
  w.requestFrom(addr, 9);
  if(w.available() >= 9) {
    int dL = w.read();
    int dH = w.read();
    for(int i=0; i<7; i++) w.read();
    return dL + (dH << 8);
  }
  return -1;
}

void doTurn(float targetAngle, int lSpd, int rSpd) {
  yaw = 0;
  while(abs(yaw) < targetAngle) {
    updateIMU();
    mc.setSpeed(1, lSpd);
    mc.setSpeed(2, rSpd);
  }
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  Serial.println("turn done");
}

void loop() {
  updateIMU();

  int btn = digitalRead(killBtn);
  if(btn == HIGH && lastBtn == LOW) {
    isStopped = !isStopped;
    if(isStopped) {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
      Serial.println("manual kill active");
    } else {
      Serial.println("robot live");
    }
    delay(200); 
  }
  lastBtn = btn;

  if(isStopped) {
    digitalWrite(ledGreen, LOW);
    if(millis() - lastBlink > 250) {
      ledState = !ledState;
      digitalWrite(ledRed, ledState);
      lastBlink = millis();
    }
  } else {
    digitalWrite(ledRed, LOW); 
    digitalWrite(ledGreen, HIGH);
  }

  if(!isStopped && Serial.available() > 0) {
    char c = Serial.read();
    
    if(c == '1') {
      mc.setSpeed(1, 200);
      mc.setSpeed(2, 200);
      Serial.println("fwd slow");
    }
    else if(c == '2') {
      mc.setSpeed(1, 600);
      mc.setSpeed(2, 600);
      Serial.println("fwd fast");
    }
    else if(c == '3') doTurn(90, -400, 400); 
    else if(c == '4') doTurn(90, 400, -400); 
    else if(c == '5') doTurn(180, 400, -400);
    else if(c == 'x') {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
    }
    else if(c == 'p') {
      currentServoIdx = (currentServoIdx + 1) % 5;
      myServo.write(servoPos[currentServoIdx]);
      Serial.print("servo moved to: ");
      Serial.println(servoPos[currentServoIdx]);
    }
    else if(c == 'e') {
      Serial.print("enc1 (L): "); Serial.print(pos1);
      Serial.print(" | enc2 (R): "); Serial.println(pos2);
    }
    else if(c == 'l') {
      Serial.print("lines: ");
      for(int i=0; i<9; i++) {
        Serial.print(digitalRead(linePins[i]));
        Serial.print(" ");
      }
      Serial.println();
    }
    else if(c == 'd') {
      int lDist = getLidar(Wire, 0x12);
      int rDist = getLidar(Wire1, 0x13);
      Serial.print("Lidar L(0x12): "); Serial.print(lDist);
      Serial.print(" R(0x13): "); Serial.println(rDist);
    }
    else if(c == 'r') {
      Serial.println("hold tag near reader...");
      unsigned long st = millis();
      bool found = false;
      while(millis() - st < 5000) {
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
          Serial.print("UID: ");
          for (byte i = 0; i < mfrc522.uid.size; i++) {
            if(mfrc522.uid.uidByte[i] < 0x10) Serial.print("0");
            Serial.print(mfrc522.uid.uidByte[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          mfrc522.PICC_HaltA();
          found = true;
          break;
        }
      }
      if(!found) Serial.println("no tag seen");
    }
  }
}