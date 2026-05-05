#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tfl;

int16_t dist = 0;
int16_t flux = 0;
int16_t temp = 0;

void setup() {
  delay(5000);
  Serial.begin(115200);
  Wire.begin();

  Serial.println("booting tf-luna...");

  if(tfl.Soft_Reset(0x10)) {
    Serial.println("reset ok");
  } else {
    Serial.println("reset failed - ground pin 5 and check wires");
  }
  delay(500);
}

void loop() {
  if(tfl.getData(dist, flux, temp, 0x10)) {
    Serial.print("dist: ");
    Serial.print(dist);
    Serial.print(" | flux: ");
    Serial.print(flux);
    Serial.print(" | temp: ");
    Serial.println(temp / 100);
  } else {
    Serial.println("read error");
    tfl.printStatus();
  }
  delay(50);
}   