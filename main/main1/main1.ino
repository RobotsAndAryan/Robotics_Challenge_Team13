/*Wiring Architecture : 
 Reflectance (Line-Following) : D22-D30(Arrays), D11-12(Control pins) , 3V3!! GND
 RFID (Tag Scanning): SDA SCL,5V GND
 2 * TF-Luna Lidar (Side Wall detection)(!! Need to change one's address):  SDA SCL, 5V GND, IIC(port 5 for I2C mode):GND
 Motoron M3S550 : Just stack it on
 2 * n20 micro metal gearmotor : (power to shield) , 5V GND (encoder power), D2-D5(Encoder Phase)(2 pins for each motor)
 64 ToF Imager (Front distance imager) : 3V3!! GND, SDA2 SCL2
 MPU6050 IMU (Turning error fixing using its Yaw data) :  5V GND, SDA2 SCL2

 Revival Mechanism : D6-7(Button and LED), 5V GND
 Seed Planting Servo : D9 , 5V GND
 Kill Switch : D8
*/

#include <Wire.h>
#include <Motoron.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

char ssid[] = "ENTER_WIFI_NAME";
char pass[] = "ENTER_WIFI_PASS";

MotoronI2C mc;
Adafruit_MPU6050 imu;
WiFiUDP udp;

int buttonPin = 6;
int ledPin = 7;

int linePins[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};

bool isStopped = true;
unsigned long lastBlink = 0;
bool ledState = false;

float yaw = 0;
unsigned long lastImuTime = 0;
int lastBtn = HIGH;

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  for(int i=0; i<9; i++) pinMode(linePins[i], INPUT);

  Wire.begin();
  Wire1.begin();
  Wire2.begin();

  mc.setBus(&Wire1);
  mc.setAddress(0x10);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.setMaxDeceleration(1, 400);
  mc.setMaxDeceleration(2, 400);
  mc.setMaxAcceleration(1, 400);
  mc.setMaxAcceleration(2, 400);
  mc.setPwmMode(1, 6);
  mc.setPwmMode(2, 6);

  if (!imu.begin(0x68, &Wire)) {
    Serial.println("imu fail");
  }

  WiFi.begin(ssid, pass);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 10) {
    delay(500);
    tries++;
  }
  udp.begin(8888); 
  
  Serial.println("ready. 1=fwd slow, 2=fwd fast, 3=L90, 4=R90, 5=U-Turn");
  Serial.println("l=lines, d=distance, r=rfid");
  lastImuTime = millis();
}

void updateIMU() {
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  float dt = (millis() - lastImuTime) / 1000.0;
  lastImuTime = millis();
  
  if(abs(g.gyro.z) > 0.05) {
    yaw += g.gyro.z * 57.2958 * dt; 
  }
}

int getLidar(TwoWire &w, int addr) {
  w.beginTransmission(addr);
  w.write(0x01);
  w.write(0x02);
  w.write(7);
  w.endTransmission();
  delay(10);
  w.requestFrom(addr, 9);
  if(w.available() >= 9) {
    int dL = w.read();
    int dH = w.read();
    for(int i=0; i<7; i++) w.read();
    return dL + (dH << 8);
  }
  return -1;
}

void doTurn(int targetAngle, int spd) {
  yaw = 0;
  while(abs(yaw) < targetAngle) {
    updateIMU();
    mc.setSpeed(1, -spd);
    mc.setSpeed(2, spd);
  }
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  Serial.println("turn done");
}

void loop() {
  updateIMU();

  int btn = digitalRead(buttonPin);
  if(btn == LOW && lastBtn == HIGH) {
    isStopped = !isStopped;
    if(isStopped) {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
    }
    delay(200); 
  }
  lastBtn = btn;

  int pk = udp.parsePacket();
  if(pk) {
    char buf[50];
    int len = udp.read(buf, 50);
    buf[len] = 0;
    if(String(buf).indexOf("Stop") >= 0) {
      isStopped = true;
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
      Serial.println("UDP STOP HIT");
    }
  }

  if(isStopped) {
    if(millis() - lastBlink > 250) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastBlink = millis();
    }
  } else {
    digitalWrite(ledPin, LOW); 
  }

  if(!isStopped && Serial.available() > 0) {
    char c = Serial.read();
    
    if(c == '1') {
      mc.setSpeed(1, 200);
      mc.setSpeed(2, 200);
      Serial.println("fwd slow");
    }
    else if(c == '2') {
      mc.setSpeed(1, 400);
      mc.setSpeed(2, 400);
      Serial.println("fwd fast");
    }
    else if(c == '3') doTurn(90, 600); 
    else if(c == '4') doTurn(90, -600); 
    else if(c == '5') doTurn(180, 600);
    else if(c == 'x') {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
    }
    else if(c == 'l') {
      Serial.print("lines: ");
      for(int i=0; i<9; i++) {
        Serial.print(digitalRead(linePins[i]));
        Serial.print(" ");
      }
      Serial.println();
    }
    else if(c == 'd') {
      int rDist = getLidar(Wire, 0x13);
      int lDist = getLidar(Wire1, 0x12);
      Serial.print("Lidar L: "); Serial.print(lDist);
      Serial.print(" R: "); Serial.println(rDist);
    }
    else if(c == 'r') {
      Wire1.beginTransmission(0x28); 
      if(Wire1.endTransmission() == 0) {
        Serial.println("RFID Tag ID: 4A B9 2C 11 (Simulated Data/Hardware Detected)");
      } else {
        Serial.println("RFID Not Found");
      }
    }
  }
}