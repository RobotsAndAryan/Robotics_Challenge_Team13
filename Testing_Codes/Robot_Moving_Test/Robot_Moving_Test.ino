#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int spd = 200;

void setup() {
  delay(2000);
  Serial.begin(115200);
  Wire1.begin();

  mc.setBus(&Wire1);
  mc.setAddress(0x14);

  mc.reinitialize();
  mc.clearResetFlag();
  mc.disableCommandTimeout();

  mc.setMaxDeceleration(1, 400);
  mc.setMaxDeceleration(2, 400);
  mc.setMaxAcceleration(1, 400);
  mc.setMaxAcceleration(2, 400);

  mc.setPwmMode(1, 6);
  mc.setPwmMode(2, 6);

  Serial.println("tank drive ready");
  Serial.println("w=fwd, s=rev, a=left, d=right, x=stop");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'w') {
      mc.setSpeed(1, spd);
      mc.setSpeed(2, spd);
      Serial.println("fwd");
    } 
    else if (cmd == 's') {
      mc.setSpeed(1, -spd);
      mc.setSpeed(2, -spd);
      Serial.println("rev");
    }
    else if (cmd == 'a') {
      mc.setSpeed(1, -spd);
      mc.setSpeed(2, spd);
      Serial.println("left");
    }
    else if (cmd == 'd') {
      mc.setSpeed(1, spd);
      mc.setSpeed(2, -spd);
      Serial.println("right");
    }
    else if (cmd == 'x') {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
      Serial.println("stop");
    }
  }
}