#include "MPU6050_tockn.h"
#include <Wire.h>
#include <Servo.h>


//create servo-object
Servo neMotor, seMotor, swMotor, nwMotor; 

struct Sonar{
  int pingPin;
  int echoPin;
};

Sonar bottom = {7, 6};

//Returns distance in cm, accurate to around 1cm.
int pulse(Sonar sensor){
  int duration;
  //Pulse
  digitalWrite(sensor.pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.pingPin, LOW);

  //Listen for echo in microseconds
  duration = pulseIn(sensor.echoPin, HIGH);
  return duration / 29 / 2;
}

//Scans a few times and takes the average of the results
float scanAverage(Sonar sensor){
  float avg;
  for(int i=0; i<5; i++){
    avg += pulse(sensor);
  }
  return avg / 5.0;
}

MPU6050 mpu(Wire);

long timer = 0;
float initYaw = 0;
float initPitch = 0;
float initRotation = 0;

//Save the speed of all the motors
int neSpeed = 0;
int seSpeed = 0;
int swSpeed = 0;
int nwSpeed = 0;


float xPos = 0;
float yPos = 0;

//Distance from sensor to the ground at launch site, 
//not the drone legs
float zPos = 0;
float ground = 0;

inline void arm(){
  setSpeed(0); //Sets speed variable delay(1000);
}

void setSpeed(int speed){
  int angle = map(speed, 0, 100, 0, 180); //Sets servo positions to different speeds 
  neMotor.write(angle);
  seMotor.write(angle);
  swMotor.write(angle);
  nwMotor.write(angle);
  neSpeed = speed;
  seSpeed = speed;
  swSpeed = speed;
  nwSpeed = speed;
}


void setup() {
  firstESC.attach(11);    // attached to pin 11
  secondESC.attach(10);
  thirdESC.attach(9);
  fourthESC.attach(6);
  arm();

  pinMode(bottom.pingPin, OUTPUT);
  pinMode(bottom.echoPin, INPUT);

  Serial.begin(9600);    // start serial at 9600 baud
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  //Get initial values of angles to offset from later
  initYaw = mpu.getAngleX();
  initPitch = mpu.getAngleY();
  initRotation = mpu.getAngleZ();

  ground = scanAverage(bottom);
  

  launch();
  hover(0, 30, 5000);
  land();
}

inline bool within(float value, float target, float margin){
  return target - value >= -margin && target - value <= margin
}

//The goal of this function is to take the live data and 
//proportionally change the power to each motor depending on the
//gyroscope's readings so that the readings will become closer to 0
//Assumes that the drone is already in the air
void hover(float targetRotation, float targetZ, int duration){
  long startTime = millis();
  int totalTime = 0;
  bool correctPos = false;

  //Exit the loop after the drone has hovered 
  //in the correct position for the duration
  while(correctPos && totalTime + millis() - startTime <= duration){
    zPos = scanAverage(bottom) - ground;
    Serial.print("cm from ground: ");Serial.println(zPos);
    Serial.print("Time: ");
    if(correctPos){
      Serial.println(totalTime + millis() - startTime);
    }else{
      Serial.print(totalTime);Serial.println("\tRe-acquiring position");
    }

    
    if(!correctPos && within(zPos, targetZ, 2)){
      correctPos = true;
      startTime = millis();
    }
    else if(correctPos && !within(zPos, targetZ, 2)){
      correctPos = false;
      totalTime += millis() - startTime;
    }

    if(!correctPos) moveTo(0, 0, targetZ, true, targetRotation);
  }
  
}


//Takes a target position that the drone will navigate to.
//This function is meant to be used in conjunction with a larger
//navigation function. Only use this in small doses.
//If halt is true, then the drone will slow down and stop at the target,
//otherwise it will not decelerate after the target is reached.
void moveTo(float targetX, float targetY, float targetZ, bool halt, float targetRotation = 0){
  //Check if the current position is at the target position
  //for each axis
  int dSpeedX = 0;
  int dSpeedY = 0;
  int dSpeedZ = 0;
  int dRotation = 0;

  int new_nwSpeed = nwSpeed;
  int new_swSpeed = swSpeed;
  int new_neSpeed = neSpeed;
  int new_seSpeed = seSpeed;

  do{
    delay(50);
    mpu.update();
    float yaw = mpu.getAngleX(); 
    float pitch = mpu.getAngleY();
    float rotation = mpu.getAngleZ();
    zPos = scanAverage(bottom) - ground;
    
    //If it is not within 1 cm or 1 degree of the target, 
    //then we need to change motor speeds
    if(!within(yaw, 0, 2)){
      dSpeedX = yaw/5;
      //If the change in speed would be over 5%, then cap it at 5%
      //but keep the sign
      if(abs(dSpeedX) > 5) dSpeedX = 5 * (abs(dSpeedX)/dSpeedX);
      //If the speed gets zeroed out, set it to one and keep the sign
      if(dSpeedX == 0) dSpeedX = abs(yaw)/yaw;
    }
    if(!within(pitch, 0, 2)){
      dSpeedY = pitch/5;
      //If the change in speed would be over 5%, then cap it at 5%
      //but keep the sign
      if(abs(dSpeedY) > 5) dSpeedY = 5 * (abs(dSpeedY)/dSpeedY);
      //If the speed gets zeroed out, set it to one and keep the sign
      if(dSpeedY == 0) dSpeedY = abs(pitch)/pitch;
    }
    if(!within(rotation, targetRotation, 2)){
      dRotation = (rotation - targetRotation)/5;
      if(abs(dRotation) > 5) dRotation = 5 * (abs(dRotation)/dRotation);
      //If the speed gets zeroed out, set it to one and keep the sign
      if(dRotation == 0) dRotation = abs(rotation - targetRotation)/(rotation - targetRotation);
    }
    if(!within(zPos, targetZ, 2)){
      dSpeedZ = round(zPos - targetZ)/10;
      //If the change in speed would be over 10%, then cap it at 10%
      //but keep the sign
      if(abs(dSpeedZ) > 10) dSpeedZ = 10 * (abs(dSpeedZ)/dSpeedZ);
      //If the speed gets zeroed out, set it to one and keep the sign
      if(dSpeedZ == 0) dSpeedZ = abs(zPos - targetZ)/(zPos - targetZ);
    }
  
    //Update speed of motors
  
    //Update X: West motors add speed, east motors subtract speed
    // To go left speed is negative, right is positive
    new_nwSpeed = nwSpeed + dSpeedX;
    new_swSpeed = swSpeed + dSpeedX;
    new_neSpeed = neSpeed - dSpeedX;
    new_seSpeed = seSpeed - dSpeedX;
  
    //Update Y: South motors add speed, north motors subtract
    // To go backwards speed is negative, forwards is positive
    new_seSpeed = seSpeed + dSpeedY;
    new_swSpeed = swSpeed + dSpeedY;
    new_neSpeed = neSpeed - dSpeedY;
    new_nwSpeed = nwSpeed - dSpeedY;
  
    //Update Z: All motors add speed
    new_nwSpeed = nwSpeed + dSpeedZ;
    new_swSpeed = swSpeed + dSpeedZ;
    new_neSpeed = neSpeed + dSpeedZ;
    new_seSpeed = seSpeed + dSpeedZ;

    //Update Rotation: Clockwise and counterclockwise
    //My guess is clockwise turns right, if it turns the wrong way
    //then just flip the signs in the next 4 lines.
    new_nwSpeed = nwSpeed - dRotation;
    new_swSpeed = swSpeed + dRotation;
    new_neSpeed = neSpeed + dRotation;
    new_seSpeed = seSpeed - dRotation;
  
    //Speeds should never be over 100
  
    //Write the speed to each motor individually
    writeSpeeds(new_neSpeed, new_seSpeed, new_swSpeed, new_nwSpeed);
    Serial.print("NW: ";Serial.print(new_nwSpeed);
    Serial.print("\tNE: ";Serial.println(new_neSpeed);
    Serial.print("SW: ";Serial.print(new_swSpeed);
    Serial.print("\tSE: ";Serial.println(new_seSpeed);
  }while(!(dSpeedX == 0 && dSpeedY == 0 && dSpeedZ == 0 && dRotation == 0));
  
}

//Write individual speeds to motors in clockwise order
void writeSpeeds(int ne, int se, int sw, int nw){
  neAngle = map(ne, 0, 100, 0, 180);
  seAngle = map(se, 0, 100, 0, 180);
  swAngle = map(sw, 0, 100, 0, 180);
  nwAngle = map(nw, 0, 100, 0, 180);
  neMotor.write(neAngle);
  seMotor.write(seAngle);
  swMotor.write(swAngle);
  nwMotor.write(nwAngle);
}


void launch(){
  int speed = 0;
  for(speed = 0; speed <= 80; speed += 5) { //Cycles speed up to 80% power for 100 millisecond
    setSpeed(speed); 
    delay(500);
    //Check if we have acheived liftoff
    if(!within(scanAverage(bottom), ground, 2)){
      //if so, then break out of the loop, we have the speed we need.
      break;
    }
  }

  Serial.println("Lift off!");
  //Check if we have reached 3 feet off the ground. (91cm)
  moveTo(0, 0, 30, true);

}

void land(){
  //Move to the ground.
  moveTo(0, 0, 0, true);
  Serial.println("Powering down...");

  //Turn off motors
  int speed;
  for(speed=(neSpeed+seSpeed+swSpeed+nwSpeed)/4; speed >= 0; speed-=5){
    setSpeed(speed);
    delay(500);
  }
  setSpeed(0);
  
}


void loop() {
  mpu.update();

}
