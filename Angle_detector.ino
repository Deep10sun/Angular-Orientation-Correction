#include <Wire.h>
#include <Servo.h>
//##############################VARS
const int MPU = 0x68;
const float PIE = 22/7.0;
float gXa,gYa,gZa; //angles for the gyro in all direction
float gXe,gYe,gZe; //errors corrections
float aX,aY,aZ,gX,gY,gZ; //acc and gyro values in all directions
float ptime,time; // used for change in time
float roll,pitch,yaw; //dimensions to move in
bool first = true;
bool sec = true;
const float gyroVar= 0.000256;
const float accVar=0.000153;
float eAngle[2];
float initP,initY;
float initErrorP,initErrorY = 4;
float measurement;
Servo p1;
Servo p2;
Servo y1;
Servo y2;
float pp;
float py;
float arrP[5]= {0.0,0.0,0.0,0.0,0.0};
float arrY[5]= {0.0,0.0,0.0,0.0,0.0};
//##############################SCOPES
void acc();
void gyro();
void get_data();
void kal_fil(float state, float w, float error,float var,float var2,float obv);
//##############################CODE
void setup(){
  p1.attach(2);
  p2.attach(3);
  y1.attach(4);
  y2.attach(5);
  Serial.begin(9600);
  //I2C
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  //ACC
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);              
  Wire.endTransmission(true);
  //GYRO
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   
  Wire.write(0x00);                     
  Wire.endTransmission(true);
  float x = 0;
  float y = 0;
  float z = 0;
  for(int i = 0; i < 2000; i++){
  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  x += (Wire.read() << 8 | Wire.read()) / 131.0; 
  y += (Wire.read() << 8 | Wire.read()) / 131.0;
  z += (Wire.read() << 8 | Wire.read()) / 131.0;
  }
  gXe = x/2000;
  gYe = y/2000;
  gZe = z/2000;
  Serial.println("STARTED");
  p1.write(90);
  p2.write(90);
  y1.write(90);
  y2.write(90);
  delay(20);
  
}
void loop(){
  int i = 0;
  int g = 0;
  acc();
  gyro();
  // measurement = (180/PIE)*(atan(aZ/aY));
  // kal_fil(initP,gX,initErrorP,gyroVar,accVar,measurement);
  // pitch = eAngle[0];
  // initErrorP = eAngle[1];
  // measurement = (180/PIE)*(atan(aX/aY));
  // kal_fil(initY,gZ,initErrorY,gyroVar,accVar,measurement);
  // yaw = eAngle[0];
  // initErrorY = eAngle[1];
  // long etime = millis()-ptime;
  
  if (abs(pitch-pp) >= 30){
    pitch = pp+0.1*gX;
  }
  else{
     pitch = (180/PIE)*(atan(aZ/aY));
  }
  if (abs(yaw-py) >= 30){
     yaw = py+0.1*gZ;
  }
  else{
    yaw = (180/PIE)*(atan(aX/aY));
  }
  if(round(pp) == round(pitch)){
    i++;
    if(i == 5){
      pitch = (180/PIE)*(atan(aZ/aY));
      i=0;
    }
  }
  else{
    i = 0;
  }
  if(py == yaw){
    g++;
    if(g == 10){
      yaw = (180/PIE)*(atan(aX/aY));
      g=0;
    }
  }
  else{
    g = 0;
  }
  // Serial.print(aX);
  // Serial.print("    ");
  // Serial.print(aY);
  // Serial.print("    ");
  // Serial.println(aZ);
  shiftR(arrY);
  shiftR(arrP);

  arrY[0] = yaw;
  float ayaw = avg(arrY);
  
  
  arrP[0] = pitch;
  float apitch = avg(arrP);

  Serial.print(pitch);
  Serial.print("    ");
  Serial.print(apitch);
  Serial.print("    ");
  Serial.print(yaw);
  Serial.print("    ");
  Serial.println(ayaw);
  p1.write(90-round(apitch+0.1*apitch));
  p2.write(90+round(apitch+0.1*apitch));
  y1.write(90+round(ayaw+0.1*ayaw));
  y2.write(90-round(ayaw+0.1*ayaw));
  py = yaw;
  pp = pitch;
  
  // Serial.print(gX);
  // Serial.print("  ");
  // Serial.print(gY);
  // Serial.print("  ");
  // Serial.println(gZ);
  delay(50);
}
void acc(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  aX = (Wire.read() << 8| Wire.read())/16384.0; // X-axis value
  aY = (Wire.read() << 8| Wire.read())/16384.0; // Y-axis value
  aZ = (Wire.read() << 8| Wire.read())/16384.0; // Z-axis value
  aX -=0.05;
  aY -=0.01;
  aZ += 0.05;
}
void gyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 
  gX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  gY = (Wire.read() << 8 | Wire.read()) / 131.0;
  gZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  gX -= gXe;
  gY -= gYe;
  gZ -= gZe;
}
void get_data(){
  for(int i = 0; i < 5000; i++){
    acc();
    Serial.print(aY);
    Serial.print(",");
  }
}
void kal_fil(float state, float w, float error,float var,float var2,float obv){
  state = state + w*0.1;
  error = error + var;
  float gain = error/(error+var2);
  state = state + gain*(obv-state);
  error = error*(1-gain);
  eAngle[0] = state;
  eAngle[1] = error;
}
void shiftR(float arr[]){
  arr[5] = arr[4];
  arr[4] = arr[3];
  arr[3] = arr[2];
  arr[2] = arr[1];
  arr[1] = arr[0];
}
float avg(float arr[]){
  float total = 0;
  for(int i = 0; i<5;i++){
    total+= arr[i];
  }
  return total/5;
}