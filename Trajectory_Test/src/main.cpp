

 

 

 

 



#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "Filters.h"
#include <ESP32Servo.h>
#include <math.h>
#include <Kalman.h>

#define TSAMPLE 1000000
const double T= 0.05;
const int MPU = 0x68; //ICD address
float AcX[2], AcY[2],AcZ[2], Tmp, GyX[2],GyY[2],GyZ[2],Vx[2],Vy[2],Vz[2],Dx=0,Dy=0,Dz=0,Zeros[3];// Dados do MPU
unsigned long currentTime=0,previousTime=0;
float ang_accelXZ,vang_gyroY,ang_kalmanXZ,ang_accelYZ,vang_gyroX,ang_kalmanYZ,ang_accelXY,vang_gyroZ,ang_kalmanXY;
float y_pass_alt[3][3], x_pass_alt[3][3];
float movAvg[10];
int count0 = 0,countMvAvg=0;
char option;
bool isFirst;

volatile int long counterAB = 0;
volatile int  laps = 0;
uint8_t dir = 0;

void ai0() {
 
  // Determina qual o sentido de giro do encoder para o contador de voltas
  if (digitalRead(35) == LOW) {
    dir = 1;
  }
  else {
    dir = 0;
  }
 
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(35) == HIGH && digitalRead(34) == LOW) {
    counterAB ++;
  }
  else {
    counterAB --;
  }
 
  if (digitalRead(35) == LOW && digitalRead(34) == HIGH) {
    counterAB ++;
  }
  else {
    counterAB --;
  }
 
}
 
// AttachInterrupt1, digital Pin 3, Sinal B - Qualquer mudança de borda (CHANGE)
void ai1() {
 
  // Determina qual o sentido de giro do encoder para o contador de voltas
  if (digitalRead(35) == HIGH) {
    dir = 1;
  }
  else {
    dir = 0;
  }
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal A
  if (digitalRead(34) == LOW && digitalRead(35) == HIGH) {
    counterAB --;
  }
  else {
    counterAB ++;
  }
 
  if (digitalRead(34) == HIGH && digitalRead(35) == LOW) {
    counterAB --;
  }
  else {
    counterAB ++;
  }
}
Kalman FiltroKalmanXZ;
Kalman FiltroKalmanYZ;
Kalman FiltroKalmanXY;
VectorFloat rawGyro;
VectorFloat rawAccel;
MPU6050 mpu;

void Setdirection(int dir);
void setupKalman()
{
  FiltroKalmanXZ.setQangle(T*T*0.0466);
  FiltroKalmanXZ.setQbias(0.0466);
  FiltroKalmanXZ.setRmeasure(10);
  FiltroKalmanXZ.setAngle(0);
  FiltroKalmanXY.setQangle(T*T*0.0466);
  FiltroKalmanXY.setQbias(0.0466);
  FiltroKalmanXY.setRmeasure(10);
  FiltroKalmanXY.setAngle(0);
  FiltroKalmanYZ.setQangle(T*T*0.0466);
  FiltroKalmanYZ.setQbias(0.0466);
  FiltroKalmanYZ.setRmeasure(10);
  FiltroKalmanYZ.setAngle(0);
}

float LPFilterAlternative(float x, int axis)
{
  
  const float a=0.50, b = 0.70;
  y_pass_alt[2][axis] = (a+b)*y_pass_alt[1][axis] -a*b*y_pass_alt[0][axis] +(1-a-b+a*b)*x_pass_alt[0][axis] ;
  float y = y_pass_alt[2][axis] ;
  y_pass_alt[0][axis] =y_pass_alt[1][axis] ;
  y_pass_alt[1][axis] =y_pass_alt[2][axis] ;
  x_pass_alt[0][axis] =x_pass_alt[1][axis] ;
  x_pass_alt[1][axis] =x_pass_alt[2][axis] ;
  x_pass_alt[2][axis] = x;
  return y;

}
float LPFilterAlternativeFirst(float x, int axis)
{
  
  const float a=0.65;
  y_pass_alt[1][axis] = (a)*y_pass_alt[0][axis]+(1-a)*x_pass_alt[1][axis];
  float y = y_pass_alt[1][axis] ;
  y_pass_alt[0][axis] =y_pass_alt[1][axis] ;
  x_pass_alt[0][axis] = x_pass_alt[1][axis] ;
  x_pass_alt[1][axis] = x;
  return y;

}
float movingAvgFilter(float x)
{
  float avg=0;
  if(countMvAvg<10)
  {
    movAvg[countMvAvg]=x;
    return x;
  }
  else
  {
    for(int i = 0 ; i<10-1; i++)
    {
      movAvg[i]=movAvg[i+1];
    }
    movAvg[9]=x;
    for(int i = 0 ; i<10-1; i++)
    {
      avg=movAvg[i]/10;
    }
    return avg;
  }
  countMvAvg++;
}
void fitData()
{
    
    accel_fit(&rawAccel.XAxis,Zeros,0);
    //rawAccel.YAxis=rawAccel.YAxis/16384;
    accel_fit(&rawAccel.YAxis,Zeros,1);
    //rawAccel.ZAxis=rawAccel.ZAxis/16384;
    accel_fit(&rawAccel.ZAxis,Zeros,2);
    
    rawGyro.XAxis=rawGyro.XAxis/131;
    rawGyro.YAxis=rawGyro.YAxis/131;
    rawGyro.ZAxis=rawGyro.ZAxis/131;
}


void velocity(float *Ac,float *velo,int *count0)
{ 
      if(abs(Ac[1])>0.1)
      {
      
        if(*velo==0)
        {
          if(isFirst&&Ac[1]<0)dir=false;
          if(isFirst&&Ac[1]>0)dir=true;         
          *velo=*velo + Ac[1]*T;
          isFirst=false;
        }
       
        if(abs(*velo+Ac[1]*T)<=0)
        {
         *velo=0;
        }    
        else
        {
         *velo=*velo + Ac[1]*T;
        }              
        
      }      
      else
      {
        
        *count0=*count0+1;
       
        if(*count0>10)
        {
          isFirst=true;
          *velo=0;
          
          *count0=0;
        }
      }
}
void displacement(float *velo,float *dis)
{ 
      if(abs(velo[1])>0.00)
      {
        if(dir)*dis=*dis +abs(velo[1])*T;
        else *dis=*dis -abs(velo[1])*T;
        
      }  
  
}

Servo dirControl;


void clearAll()
{
  AcX[1]=0;
  AcY[1]=0;
  AcZ[1]=0;
  GyX[1]=0;
  GyY[1]=0;
  GyZ[1]=0;
  Vx[1]=0;
  Vy[1]=0;
  Vz[1]=0;
  AcX[0]=0;
  AcY[0]=0;
  AcZ[0]=0;
  GyX[0]=0;
  GyY[0]=0;
  GyZ[0]=0;
  Vx[0]=0;
  Vy[0]=0;
  Vz[0]=0;
  Dx=0;
  Dy=0;
  Dz=0;
  Setdirection(0);
}
void setZero(float *Zeros, VectorFloat rawAccel)
{
  
  Zeros[0] = rawAccel.XAxis;
  Zeros[1] = rawAccel.YAxis;
  Zeros[2] = rawAccel.ZAxis;

}

void Setdirection(int dir)
{
  int _dir = 0;
  _dir = map(dir, -512, 512, 0, 180);
  dirControl.write(_dir);
}
// VectorFloat convertVectors(Vector intVector);
// VectorFloat convertVectors(Vector intVector)
// {
//   VectorFloat floatVector;
//   floatVector.XAxis=intVector.XAxis*S;
//   floatVector.YAxis=intVector.YAxis*S;
//   floatVector.ZAxis=intVector.ZAxis*S;
//   return floatVector;
  
// }

void setup() {
  // AttachInterrupt0, digital Pin 2, Sinal A
  // Ativa interrupção em qualquer mudança do sinal
  attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
 
  // AttachInterrupt1, digital pin 3, Sinal B
  // Ativa interrupção em qualquer mudança do sinal
  attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
  Serial.begin(115200);
  dirControl.attach(32);
 
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  setupKalman();
  mpu.setGyroOffsetX(84);
  mpu.setGyroOffsetY(50);
  mpu.setGyroOffsetZ(84);
}

void loop() {
  currentTime = micros();
  if (Serial.available())
  {
    option = Serial.read();
    if(option=='z')
    {
      setZero(Zeros, rawAccel);
    }
    if(option=='c')
    {
      clearAll();
     
    }
    
    
  }
 
  if (currentTime-previousTime>=T*TSAMPLE)
  {  
    rawAccel=mpu.readRawAccel();
    rawGyro=mpu.readRawGyro();

    previousTime=currentTime;

    fitData();    
    ang_accelXZ = atan2(rawAccel.XAxis, rawAccel.ZAxis)*180/PI;
    vang_gyroY=rawGyro.YAxis;
    ang_accelYZ = atan2(rawAccel.YAxis , rawAccel.ZAxis)*180/PI;
    vang_gyroX=rawGyro.XAxis;
    ang_accelXY = atan2(rawAccel.XAxis, rawAccel.YAxis )*180/PI;
    vang_gyroZ=rawGyro.ZAxis;   
     
    // Aplica o filtro de Kalman
    ang_kalmanXZ = FiltroKalmanXZ.getAngle(ang_accelXZ,vang_gyroY,T);
    ang_kalmanYZ = FiltroKalmanYZ.getAngle(ang_accelYZ,vang_gyroX,T);
    ang_kalmanXY = FiltroKalmanXY.getAngle(ang_accelXY,vang_gyroZ,T);
  
    AcY[1]=rawAccel.YAxis- sin(ang_kalmanYZ*PI/180);
    AcY[1]*=9.80665;
    AcY[1]=LPFilterAlternativeFirst(AcY[1],1);
    velocity((AcY),&Vy[1],&count0);
    displacement(Vy,&Dy);
    Serial.print(AcY[1]);Serial.print(",");
    Serial.print(Vy[1]);Serial.print(",");
    Serial.println(Dy);
    
  } 


}

