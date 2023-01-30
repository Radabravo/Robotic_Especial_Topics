

#define CarHandling_h

#include "Arduino.h"


class CarHandling
{

public:
  
 
    void SetTractionMotor(int _In1,int _In2,int _In3,int _In4);
    void SetSteeringMotor(int _SteeringPin);
    void SetSteering(int steering);
    void Move1(int Vel);
    void Move2(int Vel);
    void StopAll();
private:
    int In1,In2,In3,In4;
    int SteeringPin;
};

