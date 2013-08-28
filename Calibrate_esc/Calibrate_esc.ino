#include <Servo.h> 
Servo myservo;  

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(53);
} 
 
void loop() 
{ 
  Serial.println("start");
  myservo.write(90);
  delay(8000);
  Serial.println("forward");
  myservo.write(180);                
  delay(5000);
  Serial.println("stop");
  myservo.write(90);
  delay(500);
  Serial.println("reverse"); 
  myservo.write(0);
  delay(5000);
  Serial.println("done");
  myservo.write(90);
  delay(1000000);
} 
