/*
 * DSS-M15S Servo Motor Control - 45 Degree Increments
 * 
 * This sketch controls a DSS-M15S servo motor to rotate in 45-degree increments.
 * The servo will cycle through positions: 0°, 45°, 90°, 135°, 180°
 */

#include <Servo.h>

Servo myServo;  // Create servo object
const int servoPin = 5;  // Pin connected to servo signal wire
// Array of positions in 45-degree increments
const int positions[] = {0, 45, 90, 135, 180};
const int numPositions = 5;
int currentPosition = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);  // Attach servo to pin 9
  
  // Move to initial position
  myServo.write(positions[0]);
  Serial.println("DSS-M15S Servo Control Started");
  Serial.println("Position: 0 degrees");
  
  delay(1000);  // Wait for servo to reach position
}

void loop() {
  // Cycle through each position
  currentPosition = (currentPosition + 1) % numPositions;
  
  int angle = positions[currentPosition];
  myServo.write(angle);
  
  Serial.print("Moving to position: ");
  Serial.print(angle);
  Serial.println(" degrees");
  
  delay(2000);  // Wait 2 seconds between movements
}
