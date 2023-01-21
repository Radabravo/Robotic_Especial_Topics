#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "Filters.h"
#include <ESP32Servo.h>
#include <math.h>
#include <Kalman.h>
#include "Decode.h"
#include "AccVelDisp.h"
#include <BluetoothSerial.h>
#include "CarHandling.h"
#include <PID_v1.h>


#define TSAMPLE 1000000
#define SERIAL_TIME_INTERVAL 1000 // miliseconds
#define BT_TIME_INTERVAL 100    // miliseconds
#define BT_NUM_PACKAGES 100

BluetoothSerial serialBT;
CarHandling carHandling;
String command[2];

// Constantes do Controle PID
#define MIN_PWM 0
#define MAX_PWM 255
#define KP 5
#define KI 4
#define KD 0.002


// Variáveis do Sensor Infravermelho e PID
double Input1;
double Input2;
volatile byte pulsos;
unsigned long timeold;
double DeltaEncoder;
int pinoSensor = 2;               //Pino do Arduino ligado ao pino D0 do sensor
unsigned int pulsosDisco = 520;    //Altere o valor conforme disco encoder
double Output1 = 0;
double Output2 = 0;
double velocidadeSetpoint1 = 0;  // Alterar conforme velocidade desejadavo
double velocidadeSetpoint2 = 0;
double velocidadeSetpointToSend = 0;

// Cria PID para controle
PID motorPID1(&Input1, &Output1, &velocidadeSetpoint1, KP, KI, KD, DIRECT);
PID motorPID2(&Input2, &Output2, &velocidadeSetpoint2, KP, KI, KD, DIRECT);
unsigned long serialSendInterval;
unsigned long btSendInterval;

const int analogPin = 34;
long int analogData = 0;
uint32_t package = 0;
const double T= 0.05;
const int MPU = 0x68; //ICD address
float AcX[2], AcY[2],AcZ[2], Tmp, GyX[2],GyY[2],GyZ[2],Vx[2],Vy[2],Vz[2],Dx=0,Dy=0,Dz=0,Zeros[3];// Dados do MPU
unsigned long currentTime=0, timeTrajectory=0;
int stepCount=0,stepCount1=0;
float ang_accelXZ,vang_gyroY,ang_kalmanXZ,ang_accelYZ,vang_gyroX,ang_kalmanYZ,ang_accelXY,vang_gyroZ,ang_kalmanXY;
float y_pass_alt[3][3], x_pass_alt[3][3];
float movAvg[10];
float avgVel=0;
double trajectoryDuration = 5;
int countX=0,countY = 0,countMvAvg=0;
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
volatile int long counterAB2 = 0;
volatile int long totalCounterAB2 = 0;
volatile int  laps = 0;
bool dir = false;

//HEADERS
void setupKalman();
float LPFilterAlternative(float x, int axis);
float LPFilterAlternativeFirst(float x, int axis);
float movingAvgFilter(float x);
void fitData();
void setZero(float *Zeros, VectorFloat rawAccel);
void clearAll();
void SendData_Bluetooth();
void SendData_Serial();
int convertVelToPwm(double vel);
//HEADERS

//Decode BT
uint8_t Header1 = 0xAB;
uint8_t Header2 = 0xCD;
uint8_t Tail1 = 0xAF;
uint8_t Tail2 = 0xCF;
//Decode BT


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
void ai2() {

 
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(33) == HIGH && digitalRead(32) == LOW) {
    counterAB2 ++;
    totalCounterAB2 ++;
    dir=true;
  }
  else {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
 
  if (digitalRead(33) == LOW && digitalRead(32) == HIGH) {
    counterAB2 ++;
    totalCounterAB2 ++;
    dir=true;
  }
  else {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
 
}
 
void ai3() { 

  if (digitalRead(32) == LOW && digitalRead(33) == HIGH) {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
  else {
    counterAB2 ++;
    totalCounterAB2 ++;
    dir=true;
  }
 
  if (digitalRead(32) == HIGH && digitalRead(33) == LOW) {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
  else {
    counterAB2 ++;
    totalCounterAB2++;
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
  int avgVeltoSend = (int)(abs(Input1*(2*PI*0.033/520))*1000);
  int calculateDisplacement = (int)(trajectoryDuration*velocidadeSetpointToSend*1000); 
  int sendDy = (int)(abs(Dy)*10000); 
  int sendDx = (int)(abs(Dx)*10000); 
  uint8_t data1 = avgVeltoSend & 0xFF;        
  uint8_t data2 = (avgVeltoSend >> 8) & 0xFF;
  uint8_t data3 = calculateDisplacement& 0xFF;
  uint8_t data4 = (calculateDisplacement >> 8) & 0xFF;  
  uint8_t data5 = sendDy & 0xFF;        
  uint8_t data6 = (sendDy >> 8) & 0xFF;
  uint8_t data7 = Dy>0 ? 0 : 1;
  uint8_t data8 = sendDx & 0xFF;
  uint8_t data9 = (sendDx >> 8) & 0xFF;  
  uint8_t data10 = Dx>0 ? 0 : 1; 
  uint8_t data11 = convertVelToPwm(Input1); 
  uint8_t data12 = totalCounterAB2 & 0xFF;        
  uint8_t data13 = (totalCounterAB2 >> 8) & 0xFF;; 
  uint8_t data14 = (totalCounterAB2 >> 16)& 0xFF; 
  uint8_t data15 = (totalCounterAB2 >> 24) & 0xFF; 
  uint8_t data16 = totalCounterAB & 0xFF;        
  uint8_t data17 = (totalCounterAB >> 8) & 0xFF;; 
  uint8_t data18 = (totalCounterAB >> 16)& 0xFF; 
  uint8_t data19 = (totalCounterAB >> 24) & 0xFF; 

  if (millis() - btSendInterval >= BT_TIME_INTERVAL)
  {
    uint8_t data[23];

    data[0] = 0xAB;
    data[1] = 0xCD;
    data[2] = data1;
    data[3] = data2;
    data[4] = data3;
    data[5] = data4;
    data[6] = data5;
    data[7] = data6;
    data[8] = data7;
    data[9] = data8;    
    data[10] = data9;
    data[11] = data10;
    data[12] = data11;
    data[13] = data12;
    data[14] = data13;
    data[15] = data14;
    data[16] = data15;
    data[17] = data16;
    data[18] = data17;
    data[19] = data18;
    data[20] = data19;
    data[21] = 0xAF;
    data[22] = 0xCF;
   

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
    
    // rawGyro.XAxis=rawGyro.XAxis/131;
    // rawGyro.YAxis=rawGyro.YAxis/131;
    // rawGyro.ZAxis=rawGyro.ZAxis/131;
}





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
  //carHandling.SetSteering(90);
  counterAB=0;
  counterAB2=0;

}
void setZero(float *Zeros, VectorFloat rawAccel)
{
  
  Zeros[0] = rawAccel.XAxis;
  Zeros[1] = rawAccel.YAxis;
  Zeros[2] = rawAccel.ZAxis;

}



int convertVelToPwm(double vel)
{
  
  // int pwm = (int)((vel+0.2333)/0.0130);
  
  
  
  int pwm = (int)((vel+2917)/163);
  
  if (pwm<0)
  {
    pwm=0;
  }
  
  return pwm;
 
}




void setup() {
  //carHandling.StopAll();
  motorPID1.SetOutputLimits(MIN_PWM, 1000000);
  motorPID2.SetOutputLimits(MIN_PWM, 1000000);
  motorPID1.SetMode(AUTOMATIC);
 
  motorPID2.SetMode(AUTOMATIC);
  attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(32), ai2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(33), ai3, CHANGE);
  Serial.begin(9600);
  carHandling.SetSteeringMotor(25);
  velocidadeSetpoint1=0;
  velocidadeSetpoint2=0;
  carHandling.SetTractionMotor(IN1,IN2,IN3,IN4);
  carHandling.SetSteering(90);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  setupKalman();
  mpu.setGyroOffsetX(84);
  mpu.setGyroOffsetY(50);
  mpu.setGyroOffsetZ(84);
 
  serialBT.begin("ESP32");
  serialSendInterval = millis();
  btSendInterval = millis();
  carHandling.StopAll();
}

void loop() {
  int sizeDecoded = 14;
  uint8_t dataReceived[sizeDecoded+4];
  uint8_t dataDecoded[sizeDecoded];
  
  if(serialBT.available())
  {
    serialBT.readBytes(dataReceived, sizeDecoded+4);
    GetDecodeData(dataReceived, dataDecoded, sizeDecoded, Header1, Header2, Tail1, Tail2);
    
    command[0] = char(dataDecoded[0]);
    command[1] = char(dataDecoded[1]);
    int timeSet = int((unsigned char)(dataDecoded[2]) << 24 |
            (unsigned char)(dataDecoded[3]) << 16 |
            (unsigned char)(dataDecoded[4]) << 8 |
            (unsigned char)(dataDecoded[5]));
    int angleSet = int((unsigned char)(dataDecoded[6]) << 24 |
            (unsigned char)(dataDecoded[7]) << 16 |
            (unsigned char)(dataDecoded[8]) << 8 |
            (unsigned char)(dataDecoded[9]));
    int velSet = int((unsigned char)(dataDecoded[10]) << 24 |
            (unsigned char)(dataDecoded[11]) << 16 |
            (unsigned char)(dataDecoded[12]) << 8 |
            (unsigned char)(dataDecoded[13]));
    trajectoryDuration = (double(timeSet)/1000);
    //Serial.println(angleSet);
    carHandling.SetSteering(angleSet);
    if (command[0]+command[1]=="rn")
    {
      
      
      
      counterAB=0;
      totalCounterAB=0;
      counterAB2=0;
      totalCounterAB2=0;
      canRun=true;
      timeold = micros();
      timeTrajectory = timeold;
      velocidadeSetpoint1 = round(((double(velSet)/1000))*(520/(0.033*2*PI)));
      velocidadeSetpointToSend = ((double(velSet)/1000));
      velocidadeSetpoint2 = velocidadeSetpoint1;
      // velocidadeSetpoint1 = ((double(velSet)/1000));
      // velocidadeSetpointToSend = ((double(velSet)/1000));
      // velocidadeSetpoint2 = velocidadeSetpoint1;
      
      clearAll();
     
     
    }
    
    
    else if (command[0]+command[1]=="st")
    {
      canRun=false;
      velocidadeSetpoint1 = 0;    
      velocidadeSetpoint2 = 0;
      velocidadeSetpointToSend = velocidadeSetpoint1; 
      timeold = micros();
      timeTrajectory = timeold;
      counterAB = 0;
      counterAB2=0;     
      
      clearAll();
    }
    else 
    {
      canRun = false;
      velocidadeSetpoint1 = 0;    
      velocidadeSetpoint2 = 0;
      velocidadeSetpointToSend = velocidadeSetpoint1; 
      timeold = micros();
      timeTrajectory = timeold;
      counterAB = 0;
      totalCounterAB = 0;
      counterAB2=0;
      totalCounterAB2=0;
      clearAll();
    }
    

  }
   
  if (micros()-timeTrajectory>=trajectoryDuration*TSAMPLE && canRun)
  {
    timeTrajectory=micros();
    velocidadeSetpoint1=0;
    velocidadeSetpoint2 = 0;
    canRun=false;
  }
  
  if (micros()-currentTime>=T*TSAMPLE)
  {  
    // if(canRun)
    // {
      
    //   stepCount++;
    
      
    
      
    //   if (stepCount>=5/T)
    //   {
    //     //carHandling.StopAll();
    //     canRun=false;
    //     stepCount=0;
        
        
    //   }
      
      
    // }
   
    rawAccel=mpu.readRawAccel();
    rawGyro=mpu.readNormalizeGyro();

    currentTime=micros();

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
  
   
    AcX[1]*=9.80665;
    AcY[1]*=9.80665;
    AcY[1]=LPFilterAlternativeFirst(AcY[1],1);
    AcX[1]=LPFilterAlternativeFirst(AcX[1],1);
    AcY[1]=rawAccel.YAxis- sin(ang_kalmanYZ*PI/180);
    AcX[1]= rawAccel.XAxis - sin(ang_kalmanXZ*PI/180);
    //float avgVY= movingAvgFilter(AcY[1]);
    velocity((AcY),&Vy[1],&countY, T, canRun);
    velocity((AcX), &Vx[1],&countX, T, canRun);
    displacement(Vy,&Dy, T, dir);
    if (vang_gyroZ>0)
    {
      displacement(Vx,&Dx, T, true);
    }
    else   displacement(Vx,&Dx, T, false);
    
    
    Serial.print(Output1);Serial.print(",");
    Serial.print(Output2);Serial.print(",");
    // Serial.print(DeltaEncoder);Serial.print(",");
    Serial.print(velocidadeSetpoint1);Serial.print(",");
    Serial.print(velocidadeSetpoint2);Serial.print(",");
    Serial.println(Input1);
    
    // Serial.println(DeltaEncoder);
    // Serial.print(ang_kalmanYZ);Serial.print(",");
    // Serial.println(ang_kalmanXZ);
    // Serial.println("Total Pulses: ");
    // Serial.println(((totalCounterAB)));
    // Serial.println(((totalCounterAB2)));
    // Serial.println("Average speed: ");
    //Serial.println(((avgVel)), 5);
   
    SendData_Bluetooth();
    
  } 

  
  
  motorPID1.Compute();
  motorPID2.Compute();
  // Ajusta PWM no motor
  carHandling.Move1(convertVelToPwm (Output1));   // Utiliza velocidade calculada
  carHandling.Move2(convertVelToPwm (Output2));

  if (micros()-timeold>=100000)
  {
    detachInterrupt(34);
    detachInterrupt(35);    //Desabilita interrupção durante o cálculo para evitar sair do IF
    detachInterrupt(25);
    detachInterrupt(33);
    
    // Input1 = ((2*PI*0.033*counterAB/52));
    // Input2 = ((2*PI*0.033*counterAB2/52));
    Input1 = ((counterAB*10));
    Input2 = ((counterAB2*10));
    DeltaEncoder = Input1-Input2;
    //Serial.println(DeltaEncoder);
    // if (DeltaEncoder>=0)
    // {
    //   velocidadeSetpoint1 = velocidadeSetpoint1 - (DeltaEncoder)*0.1;
    //   velocidadeSetpoint2 = velocidadeSetpoint2 + (DeltaEncoder)*0.1;
      
    // }
    // else 
    // {
    //   velocidadeSetpoint1 = velocidadeSetpoint1 + (abs(DeltaEncoder))*0.1;
    //   velocidadeSetpoint2 = velocidadeSetpoint2 + (DeltaEncoder)*0.1;
    
    //Serial.print("Teste");
    // }
    
    timeold = micros();
    pulsos = 0;
   
   
    counterAB = 0;
    counterAB2 = 0;

    
    // Habilita novamente a interrupção
    attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(32), ai2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(33), ai3, CHANGE);
  }

}
