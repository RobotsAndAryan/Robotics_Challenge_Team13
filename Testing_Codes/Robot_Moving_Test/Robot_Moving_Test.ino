#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int spd = 600;

int enc1A = 3;
int enc1B = 4;
int enc2A = 11;
int enc2B = 12;

volatile long pos1 = 0;
volatile long pos2 = 0;

long last_pos1 = 0;
long last_pos2 = 0;
unsigned long last_time = 0;

void tick1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++;
  else pos1--;
}

void tick2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++;
  else pos2--;
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  Wire1.begin();

  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), tick1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), tick2, CHANGE);

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
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'w') {
      mc.setSpeed(1, spd);
      mc.setSpeed(2, spd);
    } 
    else if (cmd == 's') {
      mc.setSpeed(1, -spd);
      mc.setSpeed(2, -spd);
    }
    else if (cmd == 'a') {
      mc.setSpeed(1, -spd);
      mc.setSpeed(2, spd);
    }
    else if (cmd == 'd') {
      mc.setSpeed(1, spd);
      mc.setSpeed(2, -spd);
    }
    else if (cmd == 'x') {
      mc.setSpeed(1, 4);
      mc.setSpeed(2, 4);
    }
  }

  if (millis() - last_time >= 100) {
    long p1 = pos1;
    long p2 = pos2;
    
    float v1 = (p1 - last_pos1) / 0.1; 
    float v2 = (p2 - last_pos2) / 0.1;
    
    last_pos1 = p1;
    last_pos2 = p2;
    last_time = millis();

    Serial.print("L_ticks/sec:");
    Serial.print(v1);
    Serial.print(" R_ticks/sec:");
    Serial.println(v2);
  }
}