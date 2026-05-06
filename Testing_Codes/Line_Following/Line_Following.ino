#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int SensorPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
int SensorCount = 9;

const uint8_t emitterPinOdd = 11;  
const uint8_t emitterPinEven = 12; 

int spd = 500;

void navigation_and_movement(char dir){
  switch (dir){
    case 'w': 
      mc.setSpeed(1, spd); 
      mc.setSpeed(2, spd);
      break;
    case 's': 
      mc.setSpeed(1, -spd);
      mc.setSpeed(2, -spd);
      break;
    case 'a': 
      mc.setSpeed(1, spd - 100);
      mc.setSpeed(2, spd + 100);
      break;
    case 'd': 
      mc.setSpeed(1, spd + 100);
      mc.setSpeed(2, spd - 100);
      break; 
    default: 
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
  }
}

void line_detecting(uint16_t* sensorValues){
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], OUTPUT);
    digitalWrite(SensorPins[i], HIGH);
  }

  delayMicroseconds(15);

  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], INPUT);
    sensorValues[i] = 1000; 
  }

  unsigned long startTime = micros();
  while (micros() - startTime < 1000) { 
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == 1000 && digitalRead(SensorPins[i]) == LOW) {
        sensorValues[i] = micros() - startTime;
      }
    }
  }
}

void line_calibration(){
  uint16_t max_time[9] = {0};
  uint16_t min_time[9] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  uint16_t line_data[9];

  delay(1000);
  Serial.println("Line Calibration Time , slide the robot accross the line");
  
  unsigned long time_start = millis();
  while((millis() - time_start) < 3000) {
    line_detecting(line_data);
    for (int i = 0; i < SensorCount; i++) {
      if (line_data[i] > max_time[i]) max_time[i] = line_data[i];
      if (line_data[i] < min_time[i]) min_time[i] = line_data[i];
    }
  }
  Serial.println("calibration done");
}

void line_following_and_motor_correction(){
  
}

void setup(){
  delay(2000);
  Serial.begin(115200);
  Serial.println("Arduino Working ");
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
}

void loop(){
  if (Serial.available() > 0){
    char incoming = Serial.read();
    if(incoming != '\n' && incoming != '\r') {
      navigation_and_movement(incoming);
    }
  }
}