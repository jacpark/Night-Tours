
#include "MPU6050_tockn.h"
#include <Wire.h>

MPU6050 mpu6050(Wire);

long timer = 0;

struct Gyro{
  float yaw;
  float pitch;
  float rotation;
};

struct Velocity{
  float x;
  float y;
  float z;
};

Gyro orientation = {0, 0, 0};

Velocity v = {0,0,0};
Velocity vPast = {0,0,0};

Velocity initAccel = {0, 0, 0};
Veloctiy pastAccel = {0, 0, 0};

float xPos = 0;
float yPos = 0;
float zPos = 0;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  initAccel.x = mpu6050.getAccX();
  initAccel.y = mpu6050.getAccY();
  initAccel.z = mpu6050.getAccZ();

  timer = millis();

//  pinMode(pingPin, OUTPUT);
//  pinMode(echoPin, INPUT);
}

//Returns distance in cm, accurate to around 1cm.
//int pulse() {
//  int duration;
//  //Pulse
//  digitalWrite(pingPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(pingPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(pingPin, LOW);
//
//  //Listen for echo
//  duration = pulseIn(echoPin, HIGH);
//  return microseconds / 29 / 2;
//}


inline float roundToTwentieth(float n){
  return ((float)round(n * 20)) / 20.0;
}


//Returns the approximate antiderivative of two points
inline float integral(float dx, float y1, float y2){
  return dx * y1 + (y2 - y1) * dx * .5;
}


void updatePos(){
  float dTime = ((float)(millis() - timer))/1000.0;
  timer = millis();
  
  // Save orientation
  orientation.yaw = mpu6050.getAngleX();
  orientation.pitch = mpu6050.getAngleY();
  orientation.rotation = mpu6050.getAngleZ();

  //Read acceleration and subtract offsets
  //Also round to the nearest twentieth (20th or .05) to 
  //improve precision of the data, then convert Gs to m to cm
  float accX = 9.8 * 100 * roundToTwentieth(mpu6050.getAccX() - initAccel.x);
  float accY = 9.8 * 100 * roundToTwentieth(mpu6050.getAccY() - initAccel.y);
  float accZ = 9.8 * 100 * roundToTwentieth(mpu6050.getAccZ() - initAccel.z); 

  //Calculate velocity
  //Convert cm/s^2 to cm/s
  v.x += integral(dTime, pastAccel.x, accX);
  v.y += integral(dTime, pastAccel.y, accY);
  v.z += integral(dTime, pastAccel.z, accZ);

  pastAccel.x = accX;
  pastAccel.y = accY;
  pastAccel.z = accZ;

  //Calculate distance traveled, cm/s to cm
  int x = integral(dTime, vPast.x, v.x);
  int y = integral(dTime, vPast.y, v.y);
  int z = integral(dTime, vPast.z, v.z);

  vPast = v;

  //Calculate position
  float cosine = cos(orientation.rotation * PI / 180.0);
  xPos += x*cosine;
  yPos += y*cosine;
  zPos += z;

  Serial.print("x: ");Serial.println(v.x);
  Serial.print("y: ");Serial.println(v.y);
  Serial.print("z: ");Serial.println(v.z);
  Serial.print("millis: ");Serial.println(dTime * 1000);
}


void loop() {
  mpu6050.update();
  updatePos();

  

//  if(millis() - timer > 500){
//    Serial.println("=======================================================");
//    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
//    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
//    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
//    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
//  
//    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
//    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
//    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
//  
//    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
//    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
//  
//    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
//    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
//    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
//    
//    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
//    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
//    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
//    Serial.println("=======================================================\n");
//    timer = millis();
    
//  }

}
