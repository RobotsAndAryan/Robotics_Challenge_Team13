/*Wiring Architecture : 
 Reflectance (Line-Following) : D22-D30(Arrays), D11-12(Control pins) , 3V3!! GND
 RFID (Tag Scanning): SDA1 SCL1,5V GND
 2 * TF-Luna Lidar (Side Wall detection)(!! Need to change one's address):  SDA1 SCL1, 5V GND, IIC(port 5 for I2C mode):GND
 Motoron M3S550 : Just stack it on
 2 * n20 micro metal gearmotor : (power to shield) , 5V GND (encoder power), D2-D5(Encoder Phase)(2 pins for each motor)
 64 ToF Images (Front distance imager) : 3V3!! GND, SDA1 SCL1
 MPU6050 IMU (Turning error fixing using its Yaw data) :  5V GND, SDA1 SCL1

 Revival Mechanism : D6-7(Button and LED), 5V GND
 Seed Planting Servo : D8 , 5V GND
 Kill Switch : D9
*/

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
