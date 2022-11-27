#include "Arduino.h"
#include <math.h>
#include "CarHandling.h"
#include <ESP32Servo.h>

Servo steeringControl;

void CarHandling::SetTractionMotor(int _In1,int _In2,int _In3,int _In4)
{
    In1=_In1;
    In2=_In2;
    In3=_In3;
    In4=_In4;
}

void CarHandling::SetSteeringMotor(int _SteeringPin)
{
    SteeringPin=_SteeringPin;
    
    steeringControl.attach(SteeringPin);

}
void CarHandling::SetSteering(int steering)
{
    Serial.println(steering);
    steering = map(steering, -512, 512, 0, 180);
    steeringControl.write(steering);
}
void CarHandling::Move(int Vel)
{
    if (Vel>=0)
    {    
    analogWrite(In1,Vel);
    analogWrite(In3,Vel);
    }
    else
    {
    analogWrite(In2,Vel);
    analogWrite(In4,Vel);
    }
}
void CarHandling::StopAll()
{ 
    analogWrite(In1,0);
    analogWrite(In2,0);
    analogWrite(In3,0);
    analogWrite(In4,0); 
}

