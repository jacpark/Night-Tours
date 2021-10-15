#include <Servo.h>
Servo ESC;     // create servo object to control the ESC
Servo ESC2;
Servo ESC3;
Servo ESC4;

//attach arduino pins to ultrasonic sensor
#define echoPin 2
#define trigPin 3
#define echoPin2 4
#define trigPin2 5

//variables
int potValue;  // value from the analog pin
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
long duration2; // variable for the duration of sound wave travel
int distance2; // variable for the distance measurement
int distanceToGround; // variable for distance to ground
int distanceToGroundRest = 0; // variable for initial distance to ground
bool orientated = true;

int motorSpeed = 0;
int motorSpeed1 = 0;
int motorSpeed2 = 0;
int motorSpeed3 = 0;
int motorSpeed4 = 0;
int previousSpeed1 = 0;
int previousSpeed2 = 0;
int previousSpeed3 = 0;
int previousSpeed4 = 0;

void setup() {
  // Attach the ESC on pin 8-11
  ESC.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(9,1000,2000);
  ESC3.attach(10,1000,2000);
  ESC4.attach(11,1000,2000);

  //Set up ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT
  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
}
void loop() {
  
  potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
    motorSpeed = 0;
    motorSpeed1 = 0;
    motorSpeed2 = 0;
    motorSpeed3 = 0;
    motorSpeed4 = 0;
    previousSpeed1 = 0;
    previousSpeed2 = 0;
    previousSpeed3 = 0;
    previousSpeed4 = 0;
  /*
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC.write(potValue);    // Send the signal to the ESC
  ESC2.write(potValue); 
  ESC3.write(potValue); 
  ESC4.write(potValue);
  */
  //on off switch
  while (potValue >= 400){
    potValue = analogRead(A0);
//Set the distance to ground while the quad is resting
 while (distanceToGroundRest == 0){
  chirp();
  //Serial.print("Distance: ");
  //Serial.print(distance);
  //delay (1000);
  chirp2();
  //Serial.print("Distance2: ");
  //Serial.print(distance2);
  //delay(1000);
  //check that we are getting usable data, if so set the distance to ground
  if (distance > 30) {
    Serial.print("possible inacurate data, continue");
  }
  else if ( distance2 > 30){
    Serial.print("possible inacurate data, continue");
  }
  else {
  setDistanceToGroundRest();
  Serial.print("Distance to Ground: ");
  Serial.print(distanceToGroundRest);
   /* 
  motorSpeed = map(distance, 0, 30, 0, 180);
  Serial.print("speed: ");
  Serial.print(motorSpeed);
  
      if (distance > distance2){
        motorSpeed = map(distance2, 0, 30, 0, 180);
      }*/
  }
 } 
  
  //accend and hover at 10cm
   chirp();
   chirp2();
   if (orientated == true){
    if (distance <= (distanceToGroundRest + 10) && distance2 <= (distanceToGroundRest + 10)){
      if (motorSpeed1 < 29 && motorSpeed2 < 29){
        motorSpeed1++;
        motorSpeed2++;
        previousSpeed1 = motorSpeed1;
        previousSpeed2 = motorSpeed2;
        previousSpeed1 = map(motorSpeed1, 0, 30, 0, 180);
        previousSpeed2 = map(motorSpeed2, 0, 30, 0, 180);
        ESC.write(previousSpeed1);
        ESC2.write(previousSpeed2);
        ESC3.write(previousSpeed1);
        ESC4.write(previousSpeed2);
        
      }
    }
    else if (distance > (distanceToGroundRest + 10) || distance2 > (distanceToGroundRest + 10)){
      motorSpeed1--;
      motorSpeed2--;
      previousSpeed1 = motorSpeed1;
      previousSpeed2 = motorSpeed2;
      previousSpeed1 = map(motorSpeed1, 0, 30, 0, 180);
      previousSpeed2 = map(motorSpeed2, 0, 30, 0, 180);
      ESC.write(previousSpeed1);
      ESC2.write(previousSpeed2);
      ESC3.write(previousSpeed1);
      ESC4.write(previousSpeed2);
           
    }
   }
   
   //check orientation
   chirp();
   chirp2();
   if (distance < (distance2 - 3)){
    if (motorSpeed1 < 29){
    motorSpeed1++;
    if (motorSpeed2 > 0){
    motorSpeed2--;
    }
    previousSpeed1 = motorSpeed1;
    previousSpeed1 = map(motorSpeed1, 0, 30, 0, 180);
    ESC.write(previousSpeed1);
    ESC3.write(previousSpeed1);
    }
   }
   else if (distance2 < (distance - 3)){
    if (motorSpeed2 < 29){
    motorSpeed2++;
     if (motorSpeed2 > 0){
    motorSpeed1--;
     }
    previousSpeed2 = motorSpeed2;
    previousSpeed2 = map(motorSpeed2, 0, 30, 0, 180);
    ESC2.write(previousSpeed2);
    ESC4.write(previousSpeed2);
    }
   }
   chirp();
   chirp2();
      if (distance < (distance2 - 3) || distance > (distance2 + 3)){
    orientated = false;
   }
   else {
    orientated = true;
   }
   
  Serial.print("motor speed 1: ");
  Serial.print(motorSpeed1);

  Serial.print("motor speed 2: ");
  Serial.print(motorSpeed2);
  delay(500);
  }

   }

  
   

void chirp(){
   // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
}
void chirp2(){
     // Clears the trigPin condition
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration2 = pulseIn(echoPin2, HIGH);
  // Calculating the distance
  distance2 = duration2 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.print("Distance2: ");
  Serial.print(distance2);
}
void setDistanceToGroundRest(){
  //chirp all sensors and get the average
  chirp();
  chirp2();
  distanceToGround = ((distance + distance2)/2);
  distanceToGroundRest = distanceToGround;
  Serial.print("Distance to ground resting: ");
  Serial.print(distanceToGroundRest);
}
