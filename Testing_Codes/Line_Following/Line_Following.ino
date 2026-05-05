#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

int pins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};

int spd = 100;
int direction=0;

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

};

void navigation_and_movement(int direction){
  switch direction
    case 1: // Move forward
      mc.setSpeed(1,spd); 
      mc.setSpeed(2,spd);
      break;
    case 2:
      mc.setSpeed(1,-spd);
      mc.setSpeed(2, -spd);
      
}

void loop(){

};