#include "Arduino.h"
#include "Filters.h"
#include <math.h>

void accel_fit(float *input,float *Zeros,int axis)
{
    float aux=*input;
    if(axis==0)
    { 
      *input=(aux*1.005)-Zeros[axis];
    }
    if(axis==1)
    {
      *input=(aux*1.00806809)-Zeros[axis];
    } 
    if(axis==2)
    {
      *input=(aux*0.97438991)-Zeros[axis];
    }  
  
}
