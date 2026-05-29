#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  
  Serial.println("starting global i2c scan...");
}

void scan_bus(TwoWire &bus, const char* bus_name) {
  Serial.print("scanning ");
  Serial.println(bus_name);
  
  int devices = 0;
  
  for(byte addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    byte error = bus.endTransmission();
    
    if(error == 0) {
      Serial.print("found device at 0x");
      if(addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devices++;
    }
  }
  
  if(devices == 0) {
    Serial.println("bus dead. no devices.");
  }
  Serial.println("-----------------");
}

void loop() {
  scan_bus(Wire, "Wire (I2C0)");
  scan_bus(Wire1, "Wire1 (I2C1)");
  scan_bus(Wire2, "Wire2 (I2C2)");
  
  Serial.println("scan loop finished. pausing for 5s...");
  delay(5000);
}