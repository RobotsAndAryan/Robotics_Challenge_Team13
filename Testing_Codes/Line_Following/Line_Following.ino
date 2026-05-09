#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int SensorPins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
int SensorCount = 9;

const uint8_t emitterPinOdd = 51;  
const uint8_t emitterPinEven = 52; 

int enc1A = 3;
int enc1B = 4;
int enc2A = 11;
int enc2B = 12;

volatile long pos1 = 0;
volatile long pos2 = 0;
long last_pos1 = 0;
long last_pos2 = 0;

uint16_t min_time[9] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
uint16_t max_time[9] = {0};

int last_line_err = 0;
long line_integral = 0; 
float base_spd = 300; 

float target_l = 0;
float target_r = 0;

float i_l = 0;
float i_r = 0;
float kp_vel = 0.8; 
float ki_vel = 0.2; 

unsigned long last_line_time = 0;
unsigned long last_motor_time = 0;

void tick1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) pos1++;
  else pos1--;
}

void tick2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) pos2++;
  else pos2--;
}

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
  Serial.println("calibrating...");
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
  
  line_integral += err;
  if(line_integral > 50000) line_integral = 50000; 
  if(line_integral < -50000) line_integral = -50000;
  if(err == 0) line_integral = 0;
  
  float kp_line = 0.15; 
  float ki_line = 0.0005; 
  float kd_line = 0.3; 

  int turn = (err * kp_line) + (line_integral * ki_line) + ((err - last_line_err) * kd_line);
  last_line_err = err;

  target_l = base_spd + turn;
  target_r = base_spd - turn;
}

void update_motors() {
  long p1 = pos1;
  long p2 = pos2;
  
  float v1 = (p1 - last_pos1) / 0.05; 
  float v2 = (p2 - last_pos2) / 0.05;
  
  last_pos1 = p1;
  last_pos2 = p2;

  float err_l = target_l - v1;
  float err_r = target_r - v2;

  i_l += err_l * 0.05; 
  i_r += err_r * 0.05;

  if(i_l > 1000) i_l = 1000;
  if(i_l < -1000) i_l = -1000;
  if(i_r > 1000) i_r = 1000;
  if(i_r < -1000) i_r = -1000;

  float pwm_l = 0;
  float pwm_r = 0;

  if(target_l == 0) { i_l = 0; }
  else pwm_l = (err_l * kp_vel) + (i_l * ki_vel);

  if(target_r == 0) { i_r = 0; }
  else pwm_r = (err_r * kp_vel) + (i_r * ki_vel);

  if(pwm_l > 800) pwm_l = 800;
  if(pwm_l < -800) pwm_l = -800;
  if(pwm_r > 800) pwm_r = 800;
  if(pwm_r < -800) pwm_r = -800;

  mc.setSpeed(1, pwm_l);
  mc.setSpeed(2, pwm_r);
}

void setup(){
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
  mc.setAddress(0x14); 

  mc.reinitialize();
  mc.clearResetFlag();
  mc.disableCommandTimeout();

  mc.setMaxDeceleration(1, 800);
  mc.setMaxDeceleration(2, 800);
  mc.setMaxAcceleration(1, 800);
  mc.setMaxAcceleration(2, 800);

  mc.setPwmMode(1, 7);
  mc.setPwmMode(2, 7);

  pinMode(emitterPinOdd, OUTPUT);
  pinMode(emitterPinEven, OUTPUT);
  digitalWrite(emitterPinOdd, HIGH); 
  digitalWrite(emitterPinEven, HIGH); 

  line_calibration();
  
  last_line_time = millis();
  last_motor_time = millis();
}

void loop(){
  unsigned long now = millis();
  
  if (now - last_line_time >= 10) {
    follow_line();
    last_line_time = now;
  }
  
  if (now - last_motor_time >= 50) {
    update_motors();
    last_motor_time = now;
  }
}