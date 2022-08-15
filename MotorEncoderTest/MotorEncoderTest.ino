int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:   
  analogWrite(motor1pin1, 100);
  analogWrite(motor1pin2, 0);

  analogWrite(motor2pin1, 100);
  analogWrite(motor2pin2, 0);
  delay(1000);

  analogWrite(motor1pin1, 0);
  analogWrite(motor1pin2, 100);

  analogWrite(motor2pin1, 0);
  analogWrite(motor2pin2, 100);
  delay(1000);
}
