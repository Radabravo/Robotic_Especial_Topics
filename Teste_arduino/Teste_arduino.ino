

//motor_A
int IN1 = 4 ;
int IN2 = 18 ;
int vel_PS_A = 0;

//motor_B
int IN3 = 19 ;
int IN4 = 23 ;
int vel_PS_B = 0;
long timeRead = 0; 
// Servo Direção
int dir = 26;
volatile int long counterAB1 = 0;
volatile int long totalCounterAB1 = 0;
long timeTrajectory=0;
volatile int long counterAB2 = 0;
volatile int long totalCounterAB2 = 0;
int erro1=0;
int erro2=0;
int sc1 = 0;
int sc2 = 0;
int ref_PS = 2157;
float KPA = 1.5;
float KPB = KPA;
bool run1 = true;
void ai0() {

 
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(35) == HIGH && digitalRead(34) == LOW) {
    counterAB1 ++;
    totalCounterAB1 ++;
    dir=true;
  }
  else {
    counterAB1 --;
    totalCounterAB1 --;
    dir=false;
  }
 
  if (digitalRead(35) == LOW && digitalRead(34) == HIGH) {
    counterAB1 ++;
    totalCounterAB1 ++;
    dir=true;
  }
  else {
    counterAB1 --;
    totalCounterAB1 --;
    dir=false;
  }
 
}
 
void ai1() { 

  if (digitalRead(34) == LOW && digitalRead(35) == HIGH) {
    counterAB1 --;
    totalCounterAB1 --;
    dir=false;
  }
  else {
    counterAB1 ++;
    totalCounterAB1 ++;
    dir=true;
  }
 
  if (digitalRead(34) == HIGH && digitalRead(35) == LOW) {
    counterAB1 --;
    totalCounterAB1 --;
    dir=false;
  }
  else {
    counterAB1 ++;
    totalCounterAB1++;
    dir=true;
  }
}
void ai2() {

 
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(33) == HIGH && digitalRead(32) == LOW) {
    counterAB2 ++;
    totalCounterAB2 ++;
    dir=true;
  }
  else {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
 
  if (digitalRead(33) == LOW && digitalRead(32) == HIGH) {
    counterAB2 ++;
    totalCounterAB2 ++;
    dir=true;
  }
  else {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
 
}
 
void ai3() { 

  if (digitalRead(32) == LOW && digitalRead(33) == HIGH) {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
  else {
    counterAB2 ++;
    totalCounterAB2 ++;
    dir=true;
  }
 
  if (digitalRead(32) == HIGH && digitalRead(33) == LOW) {
    counterAB2 --;
    totalCounterAB2 --;
    dir=false;
  }
  else {
    counterAB2 ++;
    totalCounterAB2++;
    dir=true;
  }
}
 
void setup() {

  attachInterrupt(digitalPinToInterrupt(34), ai0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(35), ai1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(32), ai2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(33), ai3, CHANGE);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(dir, OUTPUT);
  timeRead=micros();
  timeTrajectory=micros();
  Serial.begin(9600);

   vel_PS_A = (int)((ref_PS+273.33)/33.80);
   vel_PS_B = (int)((ref_PS+269.73)/33.72);

   analogWrite(IN1,vel_PS_A);
   analogWrite(IN3,vel_PS_B);
   analogWrite(IN2,0);
   analogWrite(IN4,0);
}
void loop() {
 
  digitalWrite(dir,LOW);
  if(micros()-timeTrajectory>2000000)
  {
    ref_PS=0;
    run1=false;
    analogWrite(IN1,0);
   analogWrite(IN3,0);
    timeTrajectory=micros();
  }
  if(micros()-timeRead>=100000)
  {
    Serial.println(erro1);
    Serial.println(erro2);
    //Serial.println(counterAB1*10);
    //Serial.println(counterAB2*10);
    erro1=(ref_PS-(counterAB1*10));
    erro2=(ref_PS-(counterAB2*10));
    counterAB1 = 0;
    counterAB2 = 0;
    timeRead=micros();
  }
  if(run1){
  if(erro1>0)
  {
    sc1 = (int)(((erro1*1.5)+273.33)/33.80);
    if (sc1 > 255) sc1 = 255;
    if (sc1 < 0) sc1 = 0;
    analogWrite(IN1,sc1);
  }
  if(erro1<0)
  {
    sc1 = sc1 - (int)(((erro1*1.5)+273.33)/33.80);
    if (sc1 > 255) sc1 = 255;
    if (sc1 < 0) sc1 = 0;
    analogWrite(IN1,sc1);
  }
  
  if(erro2>0)
  {
    sc2 = (int)(((erro2*1.5)+269.73)/33.72);
    if (sc2 > 255) sc2 = 255;
    if (sc2 < 0) sc2 = 0;
    analogWrite(IN3,sc2);
  }
  if(erro2<0)
  {

    sc2 = sc2 - (int)(((erro2*1.5)+269.73)/33.72);
    if (sc2 > 255) sc2 = 255;
    if (sc2 < 0) sc2 = 0;
    analogWrite(IN3,sc2);
      
  }}
  
}
