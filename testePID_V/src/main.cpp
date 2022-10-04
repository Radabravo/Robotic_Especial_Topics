#include <Arduino.h>
#include <PID_v1.h>
#define TSAMPLE 1000000
#define KP 0.2
#define KI 0.2
#define KD 0.005

const double T= 0.05;
//motor_A
int IN1 = 4 ;
int IN2 = 18 ;
float velocidadeA = 0;
//motor_B

int IN3 = 19 ;
int IN4 = 23 ;
float velocidadeB = 0;
int MIN_PWM = 0;
int MAX_PWM = 255;
int actualPWM=0;
volatile int long counterAB = 0;
volatile int  laps = 0;
bool dir = false;bool canRun=false;
unsigned long currentTime=0,previousTime=0;
String option;
int stepCount=0;
float setPoint;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,KP,KI,KD, DIRECT);
void stopAll();
void stopAll()
{
    analogWrite(IN1,0);
    analogWrite(IN2,0);
    analogWrite(IN3,0);
    analogWrite(IN4,0);    
}
void move(int velocity);
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
void ai0() {

 
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(35) == HIGH && digitalRead(34) == LOW) {
    counterAB ++;
    dir=true;
  }
  else {
    counterAB --;
    dir=false;
  }
 
  if (digitalRead(35) == LOW && digitalRead(34) == HIGH) {
    counterAB ++;
    dir=true;
  }
  else {
    counterAB --;
    dir=false;
  }
 
}
 

void ai1() { 

  if (digitalRead(34) == LOW && digitalRead(35) == HIGH) {
    counterAB --;
    dir=false;
  }
  else {
    counterAB ++;
    dir=true;
  }
 
  if (digitalRead(34) == HIGH && digitalRead(35) == LOW) {
    counterAB --;
    dir=false;
  }
  else {
    counterAB ++;
    dir=true;
  }
}
void setup() {
 attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
 

  attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);
 
}

void loop() {
  currentTime = micros();
  if (Serial.available())
  {
    option = Serial.readStringUntil('\n');
    if(option=="zero")
    {
      
    }
    if(option=="clear")
    {
     
     
    }
    if(option=="teste")
    {
      
      actualPWM=100;
      //Output=100;
      
      Setpoint=actualPWM;
      counterAB=0;
      stopAll();
      
      move(actualPWM);
      canRun=true;
      option=" ";
     
    } 
    if(option=="teste1")
    {
      
      actualPWM=200;      
      Setpoint=actualPWM;
      counterAB=0;      
      option=" ";
     
    }   
    else option=" ";
    
    
  }
  if (currentTime-previousTime>=T*TSAMPLE)
  {  
    if(canRun)
    {
      stepCount++;
      move(Output);
      if (stepCount>=0.1/T)
      {
        detachInterrupt(34);
        detachInterrupt(35);
        Input = (0.033*counterAB/52)/0.0012+0.0567;
        myPID.Compute();
        
        // float vel=0.033*counterAB/52;
        // if (setPoint-vel<0)
        // {
        //   if (actualPWM>MIN_PWM)
        //   {
        //     actualPWM--;
        //   }
          
          
        // }
        // if (setPoint-vel>=0)
        // {
        //   if (actualPWM<MAX_PWM)
        //   {
        //     actualPWM++;
        //   }
          
          
        // }
        Serial.print(Input);
        Serial.print(",");
        Serial.print(counterAB);
        Serial.print(",");
        Serial.print(Setpoint);
        Serial.print(",");
        counterAB=0;
        stepCount=0;
        Serial.println(Output);
        attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
        
      }
      
    }
    previousTime=currentTime;
  }
}



