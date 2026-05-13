#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tfl;

int old_addr = 0x10; 
int new_addr = 0x12; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(3000);
  
  Serial.println("changing tf-luna address...");
  
  Wire.beginTransmission(old_addr);
  if (Wire.endTransmission() != 0) {
    Serial.println("nothing at 0x10. check your pin 5 to ground.");
    while(1); 
  }
  
  if(tfl.Set_I2C_Addr(new_addr, old_addr)) {
     Serial.println("ram updated to 0x12");
  } else {
     Serial.println("failed to change address");
     tfl.printStatus();
  }
  
  delay(500);
  
  if(tfl.Save_Settings(new_addr)) {
     Serial.println("saved to eeprom permanently");
  } else {
     Serial.println("failed to save");
     tfl.printStatus();
  }
  
  delay(1000);
  tfl.Soft_Reset(new_addr);
  Serial.println("sensor rebooting... address is now 0x12");
}

void loop() {
}