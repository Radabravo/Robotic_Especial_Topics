#include <ESP32Servo.h>
Servo dirControl;
int val = 0;
int dir = 0;
String c = " ";
void setup() {
  dirControl.attach(32);
  Serial.begin(115200);

}

void loop() {
  if (Serial.available()>0)
  {  
      
      val = Serial.parseInt();   
      
    
  }
  if(val>0){ dir=val;
  dir = map(dir, -512, 512, 0, 180);
  dirControl.write(dir);
  Serial.println(dir);
  delay(15);}
}
