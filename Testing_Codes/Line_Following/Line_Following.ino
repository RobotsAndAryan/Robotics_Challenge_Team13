#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
int sMin[9];
int sMax[9];

int lastErr = 0;

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("\n--- BOOT ---");

  Serial.println("starting i2c on wire1...");
  Wire1.begin();
  mc.setBus(&Wire1);
  mc.setAddress(0x14);

  Serial.println("starting motors... (if it freezes here, ur shield is disconnected)");
  mc.reinitialize();
  Serial.println("motors found");
  
  mc.clearResetFlag();
  mc.disableCommandTimeout();

  mc.setMaxAcceleration(1, 300);
  mc.setMaxDeceleration(1, 300);
  mc.setMaxAcceleration(2, 300);
  mc.setMaxDeceleration(2, 300);

  Serial.println("setting pins");
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for(int i=0; i<9; i++) {
    sMin[i] = 1000;
    sMax[i] = 0;
  }

  Serial.println("calibrating... slide tape now");
  for(int k=0; k<200; k++) {
    if(k % 50 == 0) Serial.println("calib tick..."); 
    
    int raw[9];
    readSens(raw);
    for(int i=0; i<9; i++) {
      if(raw[i] < sMin[i]) sMin[i] = raw[i];
      if(raw[i] > sMax[i]) sMax[i] = raw[i];
    }
    delay(25);
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("calib done. starting main loop");
  delay(1000);
}

void loop() {
  int raw[9];
  readSens(raw);

  long wSum = 0;
  long sum = 0;

  for(int i=0; i<9; i++) {
    int val = raw[i];
    if(val < sMin[i]) val = sMin[i];
    if(val > sMax[i]) val = sMax[i];

    long top = (long)(val - sMin[i]) * 1000;
    long bot = (long)(sMax[i] - sMin[i]);

    int norm = 0;
    if(bot != 0) {
      norm = 1000 - (top / bot); 
    }

    if(norm > 50) {
      wSum += (long)norm * (i * 1000);
      sum += norm;
    }
  }

  int pos = 4000;
  if(sum > 0) {
    pos = wSum / sum;
  }

  int err = pos - 4000;
  
  Serial.print("err: ");
  Serial.println(err);

  float kp = 0.08;
  float kd = 0.15;

  int turn = (err * kp) + ((err - lastErr) * kd);
  lastErr = err;

  int base = 6000;
  int l = base + turn;
  int r = base - turn;

  if(l > 400) l = 400;
  if(l < -200) l = -200;
  if(r > 400) r = 400;
  if(r < -200) r = -200;

  mc.setSpeed(1, base+l);
  mc.setSpeed(2, base+r);

  delay(5);
}

void readSens(int* vals) {
  for(int i=0; i<9; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  delayMicroseconds(15);

  for(int i=0; i<9; i++) {
    pinMode(pins[i], INPUT);
    vals[i] = 1000;
  }

  long st = micros();
  while(micros() - st < 1000) {
    for(int i=0; i<9; i++) {
      if(vals[i] == 1000 && digitalRead(pins[i]) == LOW) {
        vals[i] = micros() - st;
      }
    }
  }
}