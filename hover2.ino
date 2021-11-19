#include <Wire.h>
#include <TimerOne.h>
#include <Servo.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

Servo ESC1;  //
Servo ESC2;  //
Servo ESC3;  //
Servo ESC4;  // 

int potValue =0;
float gravX = 0, gravY = 0, gravZ = 1;
bool speedUp = false;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
void gravity(int16_t gx, int16_t gy, int16_t gz){
  
  //If y-axis is actually vertical, just switch all z's and y's 
  //EXCEPT for gyros and the left of final assignment to Grav

  //Rotate around y axis
  //(gravX = 0) * cos(gz) - (gravZ = 1)*sin(gz)
  float x_afterY = -sin(gz);
  //(gravX = 0) * math.sin(gz) + (gravZ = 1)*cos(gz)
  float z_afterY = cos(gz);

  // Rotate around z axis
  //x_afterY*cos(gy) + (gravY = 0)*sin(gy)
  float x_afterZ = x_afterY*cos(gy);
  //-x_afterY*sin(gy) + (gravY = 0)*cos(gy)
  float y_afterZ = -x_afterY*sin(gy);

  // Rotate around x axis
  float z_afterX = z_afterY*cos(gx) - y_afterZ*sin(gx);
  float y_afterX = z_afterY*sin(gx) + y_afterZ*cos(gx);
  
  gravX = x_afterZ;
  gravY = z_afterX;
  gravZ = y_afterX;
}



// Initial time
long int ti;
volatile bool intFlag=false;
int motorSpeed = 0;

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);

    // Attach the ESC on pin 9
  ESC1.attach(3,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(4,1000,2000);
  ESC3.attach(5,1000,2000);
  ESC4.attach(6,1000,2000); 
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
 /*
   pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  */
  
  // Store initial time
  ti=millis();
}





// Counter
long int cpt=0;
/*
void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}
*/
// Main loop, read and display data
void loop()
{

  //while (!intFlag);
  //intFlag=false;
  
  // Display time
  Serial.print (millis()-ti,DEC);
  Serial.print ("\t");

  
  // _______________
  // ::: Counter :::
  
  // Display data counter
//  Serial.print (cpt++,DEC);
//  Serial.print ("\t");
  
 
 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  float ax=-(Buf[0]<<8 | Buf[1])/8200.0;
  float ay=-(Buf[2]<<8 | Buf[3])/8200.0;
  float az= (Buf[4]<<8 | Buf[5])/8200.0;

  // Gyroscope
  float gx=-(Buf[8]<<8 | Buf[9]);
  float gy=-(Buf[10]<<8 | Buf[11]);
  float gz= (Buf[12]<<8 | Buf[13]);
  gravity(gx, gy, gz);
  
    // Display values
  
  // Accelerometer
  Serial.print (ax); 
  Serial.print ("\t");
  Serial.print (ay);
  Serial.print ("\t");
  Serial.print (az);  
  Serial.print ("\t");
    while (speedUp == false){
    //for (int i = 0; i < 120; i++){
        ESC1.write(90); 
        ESC2.write(90); 
        ESC3.write(90); 
        ESC4.write(90);
        Serial.print(!!!);
        delay(10);
    //}
    speedUp = true;
  }
//used to maintain hover (steering will be opposite)
  if (ax  > 0){
    Serial.print("tilting left");
    //turn left motors up, right motors down
    
  }
  else if(ax  < 0){
    Serial.print("tilting right");
    //turn right motors up, left motors down
  }
  if (ay  > 0){
    Serial.print("reverse");
    //turn rear motors up, front motors down
  }
  else if(ay < 0){
    Serial.print("forward");
    //turn front motors down, rear motors up
  }


  //testing
  potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  //ESC1.write(potValue); //left front
  //ESC2.write(potValue); //right front
  //ESC3.write(potValue); //rear left
  //ESC4.write(potValue); // rear right
  Serial.print(potValue);
  /*
  // Gyroscope
  Serial.print (gx,DEC); 
  Serial.print ("\t");
  Serial.print (gy,DEC);
  Serial.print ("\t");
  Serial.print (gz,DEC);  
  Serial.print ("\t");

  /*
  // _____________________
  // :::  Magnetometer ::: 

  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
  
  
  // Magnetometer
  Serial.print (mx+200,DEC); 
  Serial.print ("\t");
  Serial.print (my-70,DEC);
  Serial.print ("\t");
  Serial.print (mz-700,DEC);  
  Serial.print ("\t");
  
  */
  
  // End of line
  Serial.println("");
//  delay(100);

    
}
