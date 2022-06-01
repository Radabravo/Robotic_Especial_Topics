#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "Filters.h"

#include <math.h>
#include <Kalman.h>

#define TSAMPLE 10000
const double T= 0.005;
const int MPU = 0x68; //ICD address
float AcX[2], AcY[2],AcZ[2], Tmp, GyX[2],GyY[2],GyZ[2],Vx[2],Vy[2],Vz[2],Dx=0,Dy=0,Dz=0;// Dados do MPU
unsigned long currentTime=0,previousTime=0;
float ang_accelXZ,vang_gyroY,ang_kalmanXZ,ang_accelYZ,vang_gyroX,ang_kalmanYZ,ang_accelXY,vang_gyroZ,ang_kalmanXY;
float y_pass_alt[3][3], x_pass_alt[3][3];
float movAvg[10];
int count0 = 0,countMvAvg=0;
Kalman FiltroKalmanXZ;
Kalman FiltroKalmanYZ;
Kalman FiltroKalmanXY;
Vector rawGyro;
Vector rawAccel;
MPU6050 mpu;

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
  
  const float a=0.65, b = 0.70;
  y_pass_alt[2][axis] = (a+b)*y_pass_alt[1][axis] -a*b*y_pass_alt[0][axis] +(1-a-b+a*b)*x_pass_alt[0][axis] ;
  float y = y_pass_alt[2][axis] ;
  y_pass_alt[0][axis] =y_pass_alt[1][axis] ;
  y_pass_alt[1][axis] =y_pass_alt[2][axis] ;
  x_pass_alt[0][axis] =x_pass_alt[1][axis] ;
  x_pass_alt[1][axis] =x_pass_alt[2][axis] ;
  x_pass_alt[2][axis] = x;
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

void setup() {
  Serial.begin(115200);
  
 
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

  if (currentTime-previousTime>=T*1000000)
  {  
    rawGyro = mpu.readRawGyro();
    rawAccel = mpu.readRawAccel();    
    previousTime=currentTime;

    fitData();    
    ang_accelXZ = atan2(rawAccel.XAxis, rawAccel.ZAxis)*180/PI;
    vang_gyroY=rawGyro.YAxis;
    ang_accelYZ = atan2(rawAccel.YAxis, rawAccel.ZAxis)*180/PI;
    vang_gyroX=rawGyro.XAxis;
    ang_accelXY = atan2(rawAccel.XAxis, rawAccel.YAxis)*180/PI;
    vang_gyroZ=rawGyro.ZAxis;
    
    // Aplica o filtro de Kalman
    ang_kalmanXZ = FiltroKalmanXZ.getAngle(ang_accelXZ,vang_gyroY,T);
    ang_kalmanYZ = FiltroKalmanYZ.getAngle(ang_accelYZ,vang_gyroX,T);
    ang_kalmanXY = FiltroKalmanXY.getAngle(ang_accelXY,vang_gyroZ,T);
    // Serial.print(rawAccel.YAxis);Serial.print(",");
    // Serial.println(rawAccel.YAxis - sin(ang_kalmanYZ*PI/180));
    //AcX[1]=LPFilterAlternative(rawAccel.XAxis,0) - sin(ang_kalmanXZ*PI/180);
    AcX[1]=rawAccel.XAxis - sin(ang_kalmanXZ*PI/180);
    AcX[1]*=9.807;
    velocity((AcX),&Vx[1],&count0);
    displacement(Vx,&Dx);
    Serial.print(AcX[1]);Serial.print(",");
    Serial.print(Vx[1]);Serial.print(",");
    Serial.println(Dx);
    Vx[0]=Vx[1];
    AcX[0]=AcX[1];
    
  }
  
  // put your main code here, to run repeatedly:

}

void fitData()
{
    rawAccel.XAxis=rawAccel.XAxis/16384;
    accel_fit(&rawAccel.XAxis,0);
    rawAccel.YAxis=rawAccel.YAxis/16384;
    accel_fit(&rawAccel.YAxis,1);
    rawAccel.ZAxis=rawAccel.ZAxis/16384;
    accel_fit(&rawAccel.ZAxis,2);
    
    rawGyro.XAxis=rawGyro.XAxis/131;
    rawGyro.YAxis=rawGyro.YAxis/131;
    rawGyro.ZAxis=rawGyro.ZAxis/131;
}


void velocity(float *Ac,float *velo,int *count0)
{ 
      if(abs(Ac[1])>0.03*9.807)
      {
        *velo=*velo + Ac[1]*0.005;
       
        
      }      
      else
      {
        
        *count0=*count0+1;
       
        if(*count0>1000)
        {
          *velo=0;
          
          *count0=0;
        }
      }
}
void displacement(float *velo,float *dis)
{ 
      if(abs(velo[1])>0.00)
      {
        *dis=*dis +velo[1]*0.005;
      }  
  
}
