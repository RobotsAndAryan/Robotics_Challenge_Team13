#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int emitterOdd = 37; 
int emitterEven = 38;
int linePins[] = {30, 29, 28, 27, 26, 25, 24, 23, 22};

// Sensor 0 is on the right, so positive weight = line on right = positive error
int weights[] = {40, 30, 20, 10, 0, -10, -20, -30, -40}; 

float Kp = 10; 
float Kd = 5.0; 
int baseSpeed = 600;
float lastError = 0;

void setMotors(int l, int r) {
  // Cap speeds to prevent pulling too much current
  if(l > 800) l = 800;
  if(l < -800) l = -800;
  if(r > 800) r = 800;
  if(r < -800) r = -800;
  
  // Negative inversion kept to patch your fragile wiring
  mc.setSpeed(1, -l); 
  mc.setSpeed(3, -r); 
}

void setup() {
  Serial.begin(115200);
  Wire1.begin();

  pinMode(emitterOdd, OUTPUT); 
  pinMode(emitterEven, OUTPUT);
  digitalWrite(emitterOdd, HIGH); 
  digitalWrite(emitterEven, HIGH);

  mc.setBus(&Wire1);
  mc.setAddress(0x10);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.setPwmMode(1, 6);
  mc.setPwmMode(3, 6);
  
  delay(2000); 
}

void loop() {
  uint16_t lineVals[9];
  
  // Capacitor Discharge Read
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

  long num = 0;
  long den = 0;
  
  // Center of Mass Calculation with HIGH NOISE THRESHOLD
  for(int i=0; i<9; i++) {
    if(lineVals[i] > 900) { // Ignores the 800us floor
      num += (long)lineVals[i] * weights[i];
      den += lineVals[i];
    }
  }

  float error = 0;
  if(den != 0) {
    error = (float)num / den;
  } else {
    error = lastError; // If it loses the line, keep turning the way it was last seen
  }

  // PD Control Math
  float P = error;
  float D = error - lastError;
  float correction = (Kp * P) + (Kd * D);
  
  lastError = error;

  // Apply correction
  int leftMotor = baseSpeed + correction;
  int rightMotor = baseSpeed - correction;

  setMotors(leftMotor, rightMotor);
}