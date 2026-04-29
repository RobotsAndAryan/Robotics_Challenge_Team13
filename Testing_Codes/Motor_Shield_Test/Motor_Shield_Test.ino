#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int encA = 22; 
int encB = 24; 

volatile long pos = 0;
long last = 0;
int spd = 3000;

void setup() {
  Serial.begin(115200);
  
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA), tick, CHANGE);
  
  Wire1.begin(); 
  mc.setBus(&Wire1);
  mc.setAddress(0x14);
  
  mc.reinitialize(); 
  mc.clearResetFlag();
  mc.disableCommandTimeout(); 

  delay(2000);
  Serial.println("calib tool");
  Serial.println("w=fwd, x=rev, s=stop, 0=reset");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == 'w') {
      mc.setSpeed(1, spd); 
      Serial.println("fwd");
    } 
    else if (c == 'x') {
      mc.setSpeed(1, -spd); 
      Serial.println("rev");
    } 
    else if (c == 's') {
      mc.setSpeed(1, 0); 
      Serial.println("stop");
    } 
    else if (c == '0') {
      pos = 0; 
      Serial.println("reset");
    }
  }

  if (pos != last) {
    Serial.println(pos);
    last = pos;
    delay(20); 
  }
}

void tick() {
  int a = digitalRead(encA);
  int b = digitalRead(encB);

  if (a == b) {
    pos++; 
  } else {
    pos--; 
  }
}