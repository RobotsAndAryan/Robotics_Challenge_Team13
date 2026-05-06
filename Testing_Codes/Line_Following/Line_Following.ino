#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int SensorPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
int SensorCount = 9;

// Define both control pins
const uint8_t emitterPinOdd = 11;  
const uint8_t emitterPinEven = 12; 

int spd = 500;
int direction=0;

void navigation_and_movement(int direction){
  switch (direction){
    case 1: // Move forward
      mc.setSpeed(1,spd); 
      mc.setSpeed(2,spd);
      break;
    case 2: // Move backward
      mc.setSpeed(1,-spd);
      mc.setSpeed(2, -spd);
      break;
    case 3: // Turn Left
      mc.setSpeed(1, spd-100);
      mc.setSpeed(2,spd+100);
      break;
    case 4: // Turn right
      mc.setSpeed(1, spd+100);
      mc.setSpeed(2,spd-100);
    default: // Stay Still
      mc.setSpeed(1,0);
      mc.setSpeed(2,0);
  }
}

void line_detecting(){
  uint16_t sensorValues[SensorCount];

  // 1. Charge the capacitors on the sensor board
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], OUTPUT);
    digitalWrite(SensorPins[i], HIGH);
  }

  // Wait 15 microseconds for capacitors to fully charge
  delayMicroseconds(15);

  // 2. Switch pins to INPUT to let them discharge
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], INPUT);
    sensorValues[i] = 1000; // Default to max timeout of 1000
  }

  // 3. Measure discharge time WITHOUT disabling system interrupts
  unsigned long startTime = micros();
  while (micros() - startTime < 1000) { // 1000us absolute max timeout
    for (uint8_t i = 0; i < SensorCount; i++) {
      // If the pin drops to LOW, record how long it took
      if (sensorValues[i] == 1000 && digitalRead(SensorPins[i]) == LOW) {
        sensorValues[i] = micros() - startTime;
      }
    }
  }
}


void setup(){
  delay(2000);
  Serial.begin(115200);
  Serial.println("Arduino Working ");
  Wire.begin();
  Wire1.begin();

  mc.setAddress(0x14);
  mc.setMaxDeceleration(1,400);
  mc.setMaxDeceleration(2,400);

  mc.setMaxAcceleration(1,400);
  mc.setMaxAcceleration(2,400);

  mc.setPwmMode(1,6);
  mc.setPwmMode(2,6);

  // 2. Turn on ALL IR LEDs (Both Odd and Even banks)
  pinMode(emitterPinOdd, OUTPUT);
  pinMode(emitterPinEven, OUTPUT);
  digitalWrite(emitterPinOdd, HIGH); 
  digitalWrite(emitterPinEven, HIGH); 

};

void loop(){
  if (Serial.available()!=0){
    if(Serial.read()!=0){
      direction = Serial.read();
      navigation_and_movement(direction);
    }
  }

};