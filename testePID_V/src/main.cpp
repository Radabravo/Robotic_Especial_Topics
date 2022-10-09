#include <Arduino.h>
#include <PID_v1.h>
#define TSAMPLE 1000000
#define KP 1
#define KI 0
#define KD 0

const double T= 0.05;
//motor_A
int IN1 = 4 ;
int IN2 = 18 ;
float velocidadeA = 0;
//motor_B
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
int IN3 = 19 ;
int IN4 = 23 ;
float velocidadeB = 0;
int MIN_PWM = 0;
int MAX_PWM = 255;
int actualPWM=0;
volatile int counterAB = 0;
volatile int  laps = 0;
bool dir = false;bool canRun=false;
unsigned long currentTime=0,previousTime=0;
double elapsedTime;
String option;
int stepCount=0;
int convertVelToPwm(double vel);
double Setpoint, Input, Output;
double computePID(double inp);
PID myPID(&Input, &Output, &Setpoint,KP,0,0, DIRECT);
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
  stopAll();
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
      
      actualPWM=convertVelToPwm(0.065);
      //Output=100;
      
      Setpoint=0.065;
      counterAB=0;
      stepCount=0;
      stopAll();
      
      move(actualPWM);
      
      
      canRun=true;
      option=" ";
      previousTime=currentTime;
     
    } 
    if(option=="teste1")
    {
      
      actualPWM=convertVelToPwm(0.178);    
      Setpoint=0.178;
      counterAB=0;      
      option=" ";
     
    }
    if(option=="teste2")
    {
      
      actualPWM=convertVelToPwm(0.065);   
      Setpoint=0.065;
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
      
      if (stepCount>=1/T)
      {
        detachInterrupt(34);
        detachInterrupt(35);
        Input = ((0.033*counterAB/520));
        
        Output+=computePID(Input);
        
        //move(convertVelToPwm(Output));
        Serial.print("Values\n");
        Serial.print(Input);
        Serial.print("\n");
        Serial.print(counterAB);
        Serial.print("\n");
        Serial.print(Setpoint);
        Serial.print("\n");
        
        Serial.println(Output);
        stepCount=0;counterAB=0;
        attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
        
      }
    //Serial.print(Input);Serial.print(",");Serial.println(Output);
      
    }
    previousTime=currentTime;
  }

}
int convertVelToPwm(double vel)
{
  int pwm = (int)((vel+0.0480)/0.0023);
  //sem estar no chão
  return pwm;
  // estando no chão
  // return 0.0093*vel - 0.5250;
}
 
double computePID(double inp){     
                      //get current time
        elapsedTime = 0.1;        //compute time elapsed from previous computation
        
        error = Setpoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = 1.4*error + 0*cumError + KD*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}