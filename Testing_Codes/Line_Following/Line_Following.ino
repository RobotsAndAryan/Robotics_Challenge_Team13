#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int SensorPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
int SensorCount = 9;

const uint8_t emitterPinOdd = 11;  
const uint8_t emitterPinEven = 12; 

uint16_t min_time[9] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
uint16_t max_time[9] = {0};
int last_error = 0;
int spd = 200; 

void line_detecting(uint16_t* vals){
  for (int i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], OUTPUT);
    digitalWrite(SensorPins[i], HIGH);
  }

  delayMicroseconds(15);

  for (int i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], INPUT);
    vals[i] = 1000; 
  }

  unsigned long st = micros();
  while (micros() - st < 1000) { 
    for (int i = 0; i < SensorCount; i++) {
      if (vals[i] == 1000 && digitalRead(SensorPins[i]) == LOW) {
        vals[i] = micros() - st;
      }
    }
  }
}

void line_calibration(){
  uint16_t raw[9];
  delay(1000);
  Serial.println("calibrating... slide over line");
  
  unsigned long t = millis();
  while((millis() - t) < 3000) {
    line_detecting(raw);
    for (int i = 0; i < SensorCount; i++) {
      if (raw[i] > max_time[i]) max_time[i] = raw[i];
      if (raw[i] < min_time[i]) min_time[i] = raw[i];
    }
  }
  Serial.println("done calib");
}

void follow_line(){
  uint16_t raw[9];
  line_detecting(raw);

  long w_sum = 0;
  long sum = 0;

  for(int i = 0; i < SensorCount; i++) {
    uint16_t v = raw[i];
    if (v < min_time[i]) v = min_time[i];
    if (v > max_time[i]) v = max_time[i];

    long range = max_time[i] - min_time[i];
    long norm = 0;
    
    if (range > 0) {
      norm = 1000L - (((v - min_time[i]) * 1000L) / range); 
    }

    if (norm > 50) {
      w_sum += norm * (i * 1000L);
      sum += norm;
    }
  }

  int pos = 4000; 
  if (sum > 0) pos = w_sum / sum;

  int err = pos - 4000;
  
  float kp = 0.05; 
  float kd = 0.15; 

  int turn = (err * kp) + ((err - last_error) * kd);
  last_error = err;

  int l_spd = spd + turn;
  int r_spd = spd - turn;

  if (l_spd > 400) l_spd = 400;
  if (l_spd < -400) l_spd = -400;
  if (r_spd > 400) r_spd = 400;
  if (r_spd < -400) r_spd = -400;

  mc.setSpeed(1, l_spd);
  mc.setSpeed(2, r_spd);
}

void setup(){
  delay(2000);
  Serial.begin(115200);
  Wire.begin();
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

  pinMode(emitterPinOdd, OUTPUT);
  pinMode(emitterPinEven, OUTPUT);
  digitalWrite(emitterPinOdd, HIGH); 
  digitalWrite(emitterPinEven, HIGH); 

  line_calibration();
}

void loop(){
  follow_line();
  delay(5);
}