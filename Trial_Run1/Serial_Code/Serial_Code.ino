unsigned long t = 0;
int mode = 0; 
float yaw_fake = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() > 0) {
    char c = Serial.read();
    if(c=='1') mode=1;
    if(c=='2') mode=2;
    if(c=='3') mode=3;
    if(c=='4') mode=4;
  }

  t = millis();

  if(mode == 1) {
    int pos = 4 + 3 * sin((float)t / 600.0);
    for(int i=0; i<9; i++) {
      int d = abs(i - pos);
      if(d == 0) Serial.print(1000);
      else if(d == 1) Serial.print(800 + random(-20, 20));
      else if(d == 2) Serial.print(350 + random(-20, 20));
      else Serial.print(100 + random(-10, 10));
      Serial.print("\t");
    }
    Serial.println();
    delay(50);
  }
  else if(mode == 2) {
    int dist = 800;
    if((t / 2000) % 2 == 0) dist = 85 + random(0, 10);
    else dist = 800 + random(-20, 20);
    Serial.print("Lidar:");
    Serial.println(dist);
    delay(100);
  }
  else if(mode == 3) {
    int cycle = (t / 2000) % 4;
    
    if (cycle == 0) yaw_fake = random(-10, 10) / 10.0;
    else if (cycle == 1) yaw_fake += 2.5; 
    else if (cycle == 2) yaw_fake = 90.0 + random(-10, 10) / 10.0;
    else if (cycle == 3) yaw_fake -= 2.5;
    
    if(yaw_fake > 90 && cycle == 1) yaw_fake = 90;
    if(yaw_fake < 0 && cycle == 3) yaw_fake = 0;

    Serial.print("Yaw:");
    Serial.println(yaw_fake);
    delay(50);
  }
  else if(mode == 4) {
    if((t / 1000) % 3 == 0) {
      Serial.println("Scanning...");
    } else {
      Serial.println("UID: 4A B9 2C 11");
    }
    delay(1000);
  }
}