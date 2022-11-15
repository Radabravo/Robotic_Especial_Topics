#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "Filters.h"
#include <ESP32Servo.h>
#include <math.h>
#include <Kalman.h>
#include "AccVelDisp.h"
#include <BluetoothSerial.h>


#define TSAMPLE 1000000
#define SERIAL_TIME_INTERVAL 1000 // miliseconds
#define BT_TIME_INTERVAL 50       // miliseconds
#define BT_NUM_PACKAGES 100

BluetoothSerial serialBT;
uint8_t dataReceived;
unsigned long serialSendInterval;
unsigned long btSendInterval;
const int analogPin = 34;
int analogData = 0;
uint32_t package = 0;
const double T= 0.05;
const int MPU = 0x68; //ICD address
float AcX[2], AcY[2],AcZ[2], Tmp, GyX[2],GyY[2],GyZ[2],Vx[2],Vy[2],Vz[2],Dx=0,Dy=0,Dz=0,Zeros[3];// Dados do MPU
unsigned long currentTime=0,previousTime=0;
int stepCount=0,stepCount1=0;
float ang_accelXZ,vang_gyroY,ang_kalmanXZ,ang_accelYZ,vang_gyroX,ang_kalmanYZ,ang_accelXY,vang_gyroZ,ang_kalmanXY;
float y_pass_alt[3][3], x_pass_alt[3][3];
float movAvg[10];
float avgVel=0;
int count0 = 0,countMvAvg=0;
bool canRun=false;
String option;
Kalman FiltroKalmanXZ;
Kalman FiltroKalmanYZ;
Kalman FiltroKalmanXY;
VectorFloat rawGyro;
VectorFloat rawAccel;
MPU6050 mpu;

//motor_A
int IN1 = 4 ;
int IN2 = 18 ;
int velocidadeA = 0;
//motor_B
int IN3 = 19 ;
int IN4 = 23 ;
int velocidadeB = 0;

volatile int long counterAB = 0;
volatile int long totalCounterAB = 0;
volatile int  laps = 0;
bool dir = false;

//HEADERS
void  SetSteering(int  steering);
void setupKalman();
float LPFilterAlternative(float x, int axis);
float LPFilterAlternativeFirst(float x, int axis);
float movingAvgFilter(float x);
void fitData();
void setZero(float *Zeros, VectorFloat rawAccel);
void clearAll();
void stopAll();
void goUP(int velocity);
void goDown(int velocity);
void SendData_Bluetooth();
void SendData_Serial();
//HEADERS




void ai0() {

 
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(35) == HIGH && digitalRead(34) == LOW) {
    counterAB ++;
    totalCounterAB ++;
    dir=true;
  }
  else {
    counterAB --;
    totalCounterAB --;
    dir=false;
  }
 
  if (digitalRead(35) == LOW && digitalRead(34) == HIGH) {
    counterAB ++;
    totalCounterAB ++;
    dir=true;
  }
  else {
    counterAB --;
    totalCounterAB --;
    dir=false;
  }
 
}
 
void ai1() { 

  if (digitalRead(34) == LOW && digitalRead(35) == HIGH) {
    counterAB --;
    totalCounterAB --;
    dir=false;
  }
  else {
    counterAB ++;
    totalCounterAB ++;
    dir=true;
  }
 
  if (digitalRead(34) == HIGH && digitalRead(35) == LOW) {
    counterAB --;
    totalCounterAB --;
    dir=false;
  }
  else {
    counterAB ++;
    totalCounterAB++;
    dir=true;
  }
}
void SendData_Bluetooth()
{
  if(!serialBT.hasClient()){
    Serial.println("Not connected");
    delay(1000);
    return;
  }
  // Sending 16 bits of data over bluetooth.
  uint8_t data1 = analogData & 0xFF;        // lsb
  uint8_t data2 = (analogData >> 8) & 0xFF; // msb

  if (millis() - btSendInterval >= BT_TIME_INTERVAL)
  {
    uint8_t data[6];

    data[0] = 0xAB;
    data[1] = 0xCD;
    data[2] = data1;
    data[3] = data2;
    data[4] = 0xAF;
    data[5] = 0xCF;

    serialBT.write(data, sizeof(data));
    package++;

    if (package >= BT_NUM_PACKAGES)
    {
      // Prevent congested.
      serialBT.flush();
      package = 0;
    }

    btSendInterval = millis();
  }
}

void SendData_Serial()
{
  // Send data over serial with one second interval.
  // Just for debug.
  if (millis() - serialSendInterval >= SERIAL_TIME_INTERVAL)
  {
    serialSendInterval = millis();
    Serial.println(analogData);
  }
}
void stopAll()
{
    analogWrite(IN1,0);
    analogWrite(IN2,0);
    analogWrite(IN3,0);
    analogWrite(IN4,0);    
}

void move(int velocity)
{
    
  if (velocity>=0)
  {
    analogWrite(IN1,velocity);
    analogWrite(IN3,velocity);
  }
  else
  {
    analogWrite(IN2,velocity);
    analogWrite(IN4,velocity);
  }
    
} 

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



Servo steeringControl;


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
  SetSteering(0);
  counterAB=0;
}
void setZero(float *Zeros, VectorFloat rawAccel)
{
  
  Zeros[0] = rawAccel.XAxis;
  Zeros[1] = rawAccel.YAxis;
  Zeros[2] = rawAccel.ZAxis;

}

void SetSteering(int steering)
{  
  steering = map(steering, -512, 512, 0, 180);
  steeringControl.write(steering);
}


void setup() {

  attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
 

  attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
  Serial.begin(9600);
  steeringControl.attach(32);
 
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  setupKalman();
  mpu.setGyroOffsetX(84);
  mpu.setGyroOffsetY(50);
  mpu.setGyroOffsetZ(84);
  dataReceived = 0;
  serialBT.begin("ESP32");
  serialSendInterval = millis();
  btSendInterval = millis();
  stopAll();
}
int convertVelToPwm(double vel)
{
  // int pwm = (int)((vel+0.0480)/0.0023);
  // //sem estar no chão
  // return pwm;
  // estando no chão
  int pwm = (int)((vel+0.5250)/0.0093);
  return pwm;
}
void loop() {
  currentTime = micros();
  if (Serial.available())
  {
    option = Serial.readStringUntil('\n');
    if(option=="zero")
    {
      setZero(Zeros, rawAccel);
    }
    if(option=="clear")
    {
      clearAll();
     
    }
    if(option=="teste1")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(50);
     
    }  
    if(option=="teste2")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(75);
     
    } 
    if(option=="teste3")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(100);
     
    } 
    if(option=="teste4")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(125);
     
    }
    if(option=="teste5")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(150);
     
    } 
    if(option=="teste6")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(175);
     
    }  
    if(option=="teste7")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(200);
     
    }   
    if(option=="teste8")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(225);
     
    }
    if(option=="teste9")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(250);
     
    }   
    if(option=="teste10")
    {
      
      stopAll();
      counterAB=0;
      totalCounterAB=0;
      canRun=true;
      move(255);
     
    }   
             
    else option=" ";
    
    
  }
  
 
  if (currentTime-previousTime>=T*TSAMPLE)
  {  
    if(canRun)
    {
      
      stepCount++;
      stepCount1++;
      
      if (stepCount1==0.1/T)
      {
        avgVel=0.33*counterAB/52;
        stepCount1=0;
        counterAB=0;
      }
      
      if (stepCount>=5/T)
      {
        stopAll();
        canRun=false;
        stepCount=0;
        
      }
      
      
    }
   
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
    velocity((AcY),&Vy[1],&count0, T);
    displacement(Vy,&Dy, T, dir);
    // Serial.print(AcY[1]);Serial.print(",");
    // Serial.print(Vy[1]);Serial.print(",");
    // Serial.println(Dy);
    // Serial.println("Total Pulses: ");
    // Serial.println(((totalCounterAB)));
    // Serial.println("Average speed: ");
    // Serial.println(((avgVel)), 5);
    
  } 
  analogData = counterAB;
  SendData_Bluetooth();
  SendData_Serial();

}

