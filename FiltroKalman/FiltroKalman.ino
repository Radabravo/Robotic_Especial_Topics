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
float tmp=0;
unsigned long currentTime=0,previousTime=0;
float ang_accel,vang_gyro,ang_kalman;
Kalman FiltroKalman;
MPU6050 mpu;


void setup() {
  Serial.begin(115200);
  /*Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);*/
  FiltroKalman.setQangle(T*T*0.0466);
  FiltroKalman.setQbias(0.0466);
  FiltroKalman.setRmeasure(10);
  FiltroKalman.setAngle(0);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setGyroOffsetX(84);
  mpu.setGyroOffsetY(50);
  mpu.setGyroOffsetZ(84);
}




void loop() {
  currentTime = micros();

  if (currentTime-previousTime>=T*1000000)
  {  
    Vector rawGyro = mpu.readRawGyro();
    Vector rawAccel = mpu.readRawAccel();
    
    previousTime=currentTime;
//    Wire.beginTransmission(MPU);
//    Wire.write(0x3B); // Registrador 0x3B (ACCEL_XOUT_H)
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU,14,true);
//    AcX[1] = (Wire.read() << 8 | Wire.read()); 
//    AcY[1] = (Wire.read() << 8 |Wire.read());
//    AcZ[1] = (Wire.read() << 8 |Wire.read());
//    tmp = (Wire.read() << 8 |Wire.read());
//    GyX[1] =(Wire.read() << 8 |Wire.read());
//    GyY[1] =(Wire.read() << 8 |Wire.read());
//    GyZ[1] =(Wire.read() << 8 |Wire.read());
    
    
    //Calcula os ângulos e velocidades ângulares em torno do eixo y
//    ang_accel = atan2(AcX[1], AcZ[1])*180/PI;
//    vang_gyro=GyY[1]/131;
    //Calcula os ângulos e velocidades ângulares em torno do eixo X
   
    rawAccel.XAxis=rawAccel.XAxis/16384;
    accel_fit(&rawAccel.XAxis,0);
    rawAccel.YAxis=rawAccel.YAxis/16384;
    accel_fit(&rawAccel.YAxis,1);
    rawAccel.ZAxis=rawAccel.ZAxis/16384;
    accel_fit(&rawAccel.ZAxis,2);
    ang_accel = atan2(rawAccel.XAxis, rawAccel.ZAxis)*180/PI;
    vang_gyro=rawGyro.YAxis/131;
    // Aplica o filtro de Kalman
    ang_kalman = FiltroKalman.getAngle(ang_accel,vang_gyro,T);
    Serial.print(rawAccel.XAxis);Serial.print(",");
    Serial.println(rawAccel.XAxis - sin(ang_kalman*PI/180));
    AcX[0]=AcX[1];
    
  }
  
  // put your main code here, to run repeatedly:

}
