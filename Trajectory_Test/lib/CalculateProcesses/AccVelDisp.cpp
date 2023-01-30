#include "Arduino.h"
#include "AccVelDisp.h"
#include <math.h>
bool isFirstY = false;
bool isFirstX = false;

void velocity(float *Ac,float *velo,int *count0, double T, bool canRun)
{     
  if (!canRun)
  {
    *velo=0;
  }
  else
  {
  

    if(abs(Ac[1])>0.02)
    {
      if(*velo==0)
      {
                
        *velo=*velo + Ac[1]*T;
        
      }

      if(((*velo+Ac[1]*T)<=0.00 && *velo>0) || ((*velo+Ac[1]*T)>=0.00 && *velo<0))
      {
        *velo=0;
      }    
      else
      {
        *velo=*velo + Ac[1]*T;
      } 
      
    
      
      
      
    }      
    else
    {
      
      *count0=*count0+1;
      
      if(*count0>10)
      {
        
        *velo=0;
        
        *count0=0;
      }
    }
  }
}

void displacement(float *velo,float *dis, double T, bool dir)
{ 
      
        if(dir){
            *dis=*dis +abs(velo[1])*T;
            
        }
        else {*dis=*dis -abs(velo[1])*T;
         
        }
        
    
  
}