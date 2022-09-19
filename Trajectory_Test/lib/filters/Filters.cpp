#include "Arduino.h"
#include "Filters.h"
#include <math.h>

void accel_fit(float *input,int axis)
{
    float aux=*input;
    if(axis==0)
    { 
      *input=(aux*1.005)-0.0670;
    }
    if(axis==1)
    {
      *input=(aux*1.00806809)+0.03041711;
    } 
    if(axis==2)
    {
      *input=(aux*0.97438991)-0.09827342;
    }  
  
}
