#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define TSAMPLE 50000
const int MPU = 0x68; //ICD address
int16_t AcX[2], AcY[2],AcZ[2], Tmp, Gyx[2],GyY[2],Gyz[2],Vx[2],Vy[2],Vz[2],Dx=0,Dy=0,Dz=0;// Dados do MPU
unsigned long currentTime=0,previousTime=0;
float ang_accel;
float y_pass_alt[3][3], x_pass_alt[3][3];
int count0 = 0;
float S = 0.00006103515625;
float acc;
void MoveVector(float *vectorAddr, int tam, float value)
{
  for (size_t k = tam-1; k > 0; k--)
  {
    *(vectorAddr+k) = *(vectorAddr+k-1);
  }
  *vectorAddr = value;

}

float LPFilter(float x)
{
  static float y_pass[2] = {0,0}, x_pass[2] = {0,0};
  const float a=0.65, b = 0.70;
  float y = (a+b)*y_pass[0] -a*b*y_pass[1]+(1-a-b+a*b)*x_pass[1];
  MoveVector(y_pass,2,y);
  MoveVector(x_pass,2,x);
  return y;
}
float LPFilterAlternative(float x, int axis)
{
  
  const float a=0.65, b = 0.70;
  y_pass_alt[2][axis] = (a+b)*y_pass_alt[1][axis] -a*b*y_pass_alt[0][axis] +(1-a-b+a*b)*x_pass_alt[0][axis] ;
  float y = y_pass_alt[2][axis] ;
  y_pass_alt[0][axis] =y_pass_alt[1][axis] ;
  y_pass_alt[1][axis] =y_pass_alt[2][axis] ;
  x_pass_alt[0][axis] =x_pass_alt[1][axis] ;
  x_pass_alt[1][axis] =x_pass_alt[2][axis] ;
  x_pass_alt[2][axis] =x;
  return y;

}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

}



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
void velocity(float *Ac,float *velo,int *count0)
{ 
      if(abs(Ac[1])>0.03)
      {
        *velo=*velo + ((Ac[1]+Ac[0])/2)*0.05;
      }      
      else
      {
        
        *count0=*count0+1;
       
        if(*count0>25)
        {
          *velo=0;
          
          *count0=0;
        }
      }
}
void displacement(float velo,float *dis)
{ 
      if(abs(velo)>0.02) *dis=*dis + velo*0.05;      
  
}
void loop() {
  currentTime = micros();
  if (currentTime-previousTime>=TSAMPLE)
  {
    
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Registrador 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);
    AcX[1] = (Wire.read() << 8 | Wire.read()); 
    AcY[1] = (Wire.read() << 8 |Wire.read());
    //ang_accel = atan2(AcY,AcX) *180/PI;
    AcX[1]=AcX[1];
    //accel_fit(&AcX[1],0);
    acc =AcX[1]*S;
    //AcX[1]=LPFilterAlternative(AcX[1],0);
    //velocity((AcX),&Vx[1],&count0);
    Serial.print(acc);Serial.print(",");
    Serial.println(Vx[1]);
    AcX[0]=AcX[1];
    previousTime=currentTime;
  }
  
  // put your main code here, to run repeatedly:

}
