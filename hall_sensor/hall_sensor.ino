#include <Servo.h> 


Servo myMotor;
int motorPin = 10;
int maxSpeed = 100;
int minSpeed = 102; // lowest to start going forward
int stopSpeed = 90;
int reverseSpeed = 78;
int fullReverse = 0;
int currentSpeed = stopSpeed;
int slowDown = 1; // how much to slow down when turning
int lastSpeed = 0; // checks to make sure current speed isnt being rewritten

unsigned long lastMillis = 0;
int hallDelay = 100;
int hallInput = 13; // hall output to power
int hallInput2 = A0; // hall input to read sensor
int maxDelay = 300; // max to wait

void setup() 
{
  pinMode(hallInput,OUTPUT);
 
 
  Serial.begin(9600);
  myMotor.attach(motorPin);
  myMotor.write(stopSpeed);
}
void loop() 
{
 hallSensor();
}

void hallSensor()
{
  digitalWrite(hallInput,LOW);
  delay(hallDelay);
  digitalWrite(hallInput,HIGH);
  delay(hallDelay);
  int sensorValue = analogRead(hallInput2);
  while(sensorValue == 0 && millis() < maxDelay)
  {
    sensorValue = analogRead(hallInput2);
  }
  unsigned long mill = millis() + 2*hallDelay;
  
  digitalWrite(hallInput,LOW);
  delay(hallDelay);
  digitalWrite(hallInput,HIGH);
  delay(hallDelay);
  sensorValue = analogRead(hallInput2); 
  while(sensorValue == 0 && millis() < maxDelay + mill + 2*hallDelay)
  {
    sensorValue = analogRead(hallInput2);
    
  }
  unsigned long mill2 = millis();
  lastMillis = mill2 - mill;
  Serial.println(lastMillis);
  Serial.println();
  
} // void hallSensor

