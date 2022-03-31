#include <Wire.h>
#include <math.h>

#define TSAMPLE 50000
const int MPU = 0x68; //ICD address
int AcX, AcY,AcZ, Tmp, Gyx,GyY,Gyz;// Dados do MPU
unsigned long currentTime=0,previousTime=0;
float ang_accel;
 float y_pass_alt[3] = {0,0,0}, x_pass_alt[3] = {0,0,0};

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
float LPFilterAlternative(float x)
{
  
  const float a=0.65, b = 0.70;
  y_pass_alt[2] = (a+b)*y_pass_alt[1] -a*b*y_pass_alt[0]+(1-a-b+a*b)*x_pass_alt[0];
  float y = y_pass_alt[2];
  y_pass_alt[0]=y_pass_alt[1];
  y_pass_alt[1]=y_pass_alt[2];
  x_pass_alt[0]=x_pass_alt[1];
  x_pass_alt[1]=x_pass_alt[2];
  x_pass_alt[2]=x;
  return y;

}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

}

void loop() {
  currentTime = micros();
  if (currentTime-previousTime>=TSAMPLE)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Registrador 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);
    AcX = Wire.read() << 8 | Wire.read(); 
    AcY = Wire.read() << 8 |Wire.read();
    ang_accel = atan2(AcY,AcX) *180/PI;

    Serial.print(LPFilter(ang_accel));Serial.print(",");
    Serial.println(ang_accel);
    previousTime=currentTime;
  }
  
  // put your main code here, to run repeatedly:

}