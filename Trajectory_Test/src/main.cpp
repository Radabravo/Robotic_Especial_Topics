#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "Decode.h"
#include <BluetoothSerial.h>
#include "CarHandling.h"
#include <PID_v1.h>


#define TSAMPLE 1000000
#define SERIAL_TIME_INTERVAL 1000 // miliseconds
#define BT_TIME_INTERVAL 100    // miliseconds
#define BT_NUM_PACKAGES 100

BluetoothSerial serialBT;
CarHandling carHandling;
String command[3];

// Constantes do Controle PID
#define MIN_PWM 0
#define MAX_PWM 255
#define KP 3
#define KI 2
#define KD 0.001
#define WB 0.145 // wheel base

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
unsigned long currentTime=0, timeTrajectory=0;
int stepCount=0,stepCount1=0;

float avgVel=0;
double trajectoryDuration = 5;
int countX=0,countY = 0,countMvAvg=0;
bool canRun=false;

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

void clearAll();
void SendData_Bluetooth();
void SendData_Serial();
int convertVelToPwm(double vel, int motor);
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
  int sendDy = (int)(abs(0)*10000); 
  int sendDx = (int)(abs(0)*10000); 
  uint8_t data1 = avgVeltoSend & 0xFF;        
  uint8_t data2 = (avgVeltoSend >> 8) & 0xFF;
  uint8_t data3 = calculateDisplacement& 0xFF;
  uint8_t data4 = (calculateDisplacement >> 8) & 0xFF;  
  uint8_t data5 = sendDy & 0xFF;        
  uint8_t data6 = (sendDy >> 8) & 0xFF;
  uint8_t data7 =0;
  uint8_t data8 = sendDx & 0xFF;
  uint8_t data9 = (sendDx >> 8) & 0xFF;  
  uint8_t data10 = 0; 
  uint8_t data11 = convertVelToPwm(Output1,1); 
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








void clearAll()
{  
  //carHandling.SetSteering(90);
  counterAB=0;
  counterAB2=0;

}



int convertVelToPwm(double vel, int motor)
{
  int pwm = 0;
  
  if (motor==1) // Motor esquerdo 
  {
    pwm = (int)((vel+1366.7)/169);
  }
  else if (motor == 2) // Motor Direito
  {
    pwm = (int)((vel+1348.7)/168.6);
  }


  //int pwm = (int)((vel+0.2333)/0.0130);
  
  
  
  // int pwm = (int)((vel+2917)/163);
  
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
  
 
  serialBT.begin("ESP32");
  serialSendInterval = millis();
  btSendInterval = millis();
  carHandling.StopAll();
}

float CalculateRadius(int angleSet)
{
 
  float Radius = WB/(tan(((PI/180)*((float)angleSet-110))));
  
  
  return Radius;
}
void CalculateCircunVel(float *vel1, float *vel2, float vel, float radius)
{
    
    *vel1=vel*(1+0.08/radius);
    *vel2=vel*(1-0.08/radius); 
   
    
}
void loop() {
  int sizeDecoded = 15;
  uint8_t dataReceived[sizeDecoded+4];
  uint8_t dataDecoded[sizeDecoded];
  
  if(serialBT.available())
  {
    serialBT.readBytes(dataReceived, sizeDecoded+4);
    GetDecodeData(dataReceived, dataDecoded, sizeDecoded, Header1, Header2, Tail1, Tail2);
    
    command[0] = char(dataDecoded[0]);
    command[1] = char(dataDecoded[1]);
    command[2] = char(dataDecoded[2]);
    int timeSet = int((unsigned char)(dataDecoded[3]) << 24 |
            (unsigned char)(dataDecoded[4]) << 16 |
            (unsigned char)(dataDecoded[5]) << 8 |
            (unsigned char)(dataDecoded[6]));
    int angleSet = int((unsigned char)(dataDecoded[7]) << 24 |
            (unsigned char)(dataDecoded[8]) << 16 |
            (unsigned char)(dataDecoded[9]) << 8 |
            (unsigned char)(dataDecoded[10]));
    int velSet = int((unsigned char)(dataDecoded[11]) << 24 |
            (unsigned char)(dataDecoded[12]) << 16 |
            (unsigned char)(dataDecoded[13]) << 8 |
            (unsigned char)(dataDecoded[14]));
    trajectoryDuration = (double(timeSet)/1000);
    //Serial.println(angleSet);
    carHandling.SetSteering(angleSet);
    if (command[0]+command[1]+command[2]=="rnl")
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
 
      
      clearAll();
     
     
    }
    else if (command[0]+command[1]+command[2]=="rnc")
    {
      float velSet1=0, velSet2=0,velLinear=0;     
      float radius = CalculateRadius(angleSet);
      velLinear = 2*PI*abs(radius)/trajectoryDuration;

      CalculateCircunVel(&velSet1,&velSet2,velLinear,radius);
      Serial.println(radius);
      Serial.println(velSet1);
      Serial.println(velSet2);
      counterAB=0;
      totalCounterAB=0;
      counterAB2=0;
      totalCounterAB2=0;
      canRun=true;
      timeold = micros();
      timeTrajectory = timeold;
      velocidadeSetpoint1 = round(((double(velSet1)))*(520/(0.033*2*PI)));
      velocidadeSetpointToSend = ((double(velLinear)));
      velocidadeSetpoint2 = round(((double(velSet2)))*(520/(0.033*2*PI)));
     
      
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
  
    

  }
   
  if (micros()-timeTrajectory>=trajectoryDuration*TSAMPLE && canRun)
  {
    timeTrajectory=micros();
    velocidadeSetpoint1=0;
    velocidadeSetpoint2=0;
    canRun=false;    
  }
  
  if (micros()-currentTime>=T*TSAMPLE)
  {  
    
    

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
  carHandling.Move1(convertVelToPwm (Output1,1));   // Utiliza velocidade calculada
  carHandling.Move2(convertVelToPwm (Output2,2));
 
    
  if (command[2]=="l")
  {
    if (DeltaEncoder>=0 && velocidadeSetpoint1!=0)
    {
      velocidadeSetpoint1 = velocidadeSetpoint1 - (DeltaEncoder)*0.0001;
      velocidadeSetpoint2 = velocidadeSetpoint2 + (DeltaEncoder)*0.0001;
      
    }
    else if (DeltaEncoder<0 && velocidadeSetpoint1!=0) 
    {
      velocidadeSetpoint1 = velocidadeSetpoint1 + (abs(DeltaEncoder))*0.0001;
      velocidadeSetpoint2 = velocidadeSetpoint2 + (DeltaEncoder)*0.0001;  
    
    }
  }
  
 

  if (micros()-timeold>=100000)
  {
    detachInterrupt(34);
    detachInterrupt(35);    //Desabilita interrupção durante o cálculo para evitar sair do IF
    detachInterrupt(25);
    detachInterrupt(33);
    
   

    Input1 = ((counterAB*10));
    Input2 = ((counterAB2*10));
    DeltaEncoder = Input1 - Input2;
    
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
