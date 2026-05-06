#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 myIMU;

float pitch = 0;
float roll = 0;
float yaw = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!myIMU.begin(0x68, &Wire)) {
    Serial.println("imu dead");
    while (1){} 
  }
  
  myIMU.setAccelerometerRange(MPU6050_RANGE_8_G);
  myIMU.setGyroRange(MPU6050_RANGE_500_DEG); 
  
  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  myIMU.getEvent(&a, &g, &temp);
  
  pitch = -(atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0)/PI;
  roll = (atan2(a.acceleration.y, a.acceleration.z)*180.0)/PI;
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // gyro returns radians per second. make it degrees.
  float gyroZ = g.gyro.z * (180.0/PI);
  
  // deadband filter so static vibration doesnt spin the robot
  if(abs(gyroZ) > 1.5) {
    yaw = yaw + (gyroZ * dt);
  }
  
  Serial.print("yaw: ");
  Serial.println(yaw);
  
  delay(10);
}