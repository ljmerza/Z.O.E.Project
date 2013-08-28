/*---------------------------------------------------------------
Created by: Leonardo Merza
Version 3.2

-serial0 outputs gps info to google earth 
-serial1 reads gps info
-serial2 reads from kinect

-line sensors need to be tested and the auto-off
feature needs to be set, the whiteLine value
to turn in the track and turn motor off.

-method to backup when sonars 1 or 3 are too close
needs worked on

-need to do math on hallSensor method to convert time 
in ms to speed
----------------------------------------------------------------*/


/*---------------------------------------------------------------
Imports.
----------------------------------------------------------------*/
#include <Servo.h> 
#include <nmea.h>

/*---------------------------------------------------------------
Variables.
----------------------------------------------------------------*/
//Sonar Variables
// echo pins of each sensor
int echo1 = 30;
int echo2 = 32;
int echo3 = 34;
int echo4 = 36;
int echo5 = 38;
int echo6 = 40;
// trig pins of each sensor
int trig1 = 31;
int trig2 = 33;
int trig3 = 35;
int trig4 = 37;
int trig5 = 39;
int trig6 = 41;
// boolean values if sensor is closer than tooClose
boolean isSonarClose1 = false;
boolean isSonarClose2 = false;
boolean isSonarClose3 = false;
boolean isSonarClose4 = false;
boolean isSonarClose5 = false;
boolean isSonarClose6 = false;
// current distance of sonar sensor
float distance1 = 0;
float distance2 = 0;
float distance3 = 0;
float distance4 = 0;
float distance5 = 0;
float distance6 = 0;

int tooClose = 25; // in cm - tells when one of the six sensors is too close
int sonarWindow = 5; // in cm - window of left and right sonar sensors.
int sonarDelay = 50; // microseconds to wait for sonars
int backUpDelay = 4000;

// Line Sensor Variables
int leftLineSensor = 8; // pin of left line sensor
int rightLineSensor = 9; // pin of right line sensor
int leftLine = 0;
int rightLine = 0;
int whiteLine = 1020; // value of line sensor when crossing white line
int lineWindow = 10;
boolean turnedON = true;

// Motor Variables
Servo myMotor;
int motorPin = 10;
int maxSpeed = 118;
int minSpeed = 102; // lowest to start going forward
int stopSpeed = 90;
int reverseSpeed = 78;
int fullReverse = 0;
int currentSpeed = stopSpeed;
int slowDown = 1; // how much to slow down when turning
int lastSpeed = 0; // checks to make sure current speed isnt being rewritten

// Servo Variables
Servo myServo; // servo object
int servoPin = 9; // pin of servo
int servoStraight = 90; // set servo straight
int servoLeft = 180; // max left turn
int servoRight = 0; // max right turn
int currentServo = servoStraight; /// start current servo as straight
int turningAngle = 50; // how much to turn when turning

// Debug Variables
int delayTime = 1; // delay time of debug method

// Kinect Variables
int distanceFromKinect = 140; // distance car follows user
int val; // current value of coordinate being tracked
int xVal=255/2, yVal=255/2, zVal=1; // curernt XYZ coordinates of head tracking 
int minKinect = 75; // min value of X coordinate
int maxKinect = 120; // max value of X coordinate

//Gps Variables
NMEA gps(GPRMC);
float gpsMaxSpeed = 0.1; // in kmph

// Hall Sensor Variables
unsigned long lastMillis = 0;
int hallDelay = 100;
int hallInput = 2; // hall output to power
int hallInput2 = 0; // hall input to read sensor
int maxDelay = 3000; // max time to wait for hall senso
float downHallwaySpeed = 2000; // speed to go down hallway
float kinectSpeed = 2000; // speed to kinect following
float lineSpeed = 2000; // speed of rc car when line following

/*---------------------------------------------------------------
Swetup method.  Initializes all pins and servo/motor.
----------------------------------------------------------------*/
void setup() 
{ 
  Serial.begin(57600); // for kinect interaction
  Serial1.begin(57600); // for gps interaction
  Serial2.begin(57600); // for google earth interaction
  
  // hall sensor voltage pin to reset it.=
  pinMode(hallInput,OUTPUT);
  
  // sonars activated
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  pinMode(echo3, INPUT);
  pinMode(echo4, INPUT);
  pinMode(echo5, INPUT);
  pinMode(echo6, INPUT);
  
  pinMode(trig1, OUTPUT);
  pinMode(trig2, OUTPUT);
  pinMode(trig3, OUTPUT);
  pinMode(trig4, OUTPUT);
  pinMode(trig5, OUTPUT);
  pinMode(trig6, OUTPUT);
  
  // servo and motor activated
  myServo.attach(servoPin);
  myMotor.attach(motorPin);
  
  // set motor and servo to default values
  myMotor.write(stopSpeed);
  myServo.write(servoStraight);
} // void setup()

/*---------------------------------------------------------------
Loop method. Sends GPS info to serial2.  Gets sonar and kinect
values.  If the front sonars are close then they take over the
servo else the kinect user is followed. If front sensor is close
then stop rc car.  
----------------------------------------------------------------*/
void loop()
{  
  // gps read and write info from serial port 1 to serial port 2

  if(Serial1.available())
  {
    Serial.write(Serial1.read());
  }

  // get x/y/z coordinates from kinect
  getKinectValues(); 

  checkFrontSonars();
  //checkBackSonars();
  
  if(isSonarClose2)
  {
    hardStop();
  }
  else if(isSonarClose1 | isSonarClose3)
  {
    sonarDirect();
  } // if sonar 1 or 3 is close then take over
  else
  {
    kinectFollowX(); // servo follows user on kinect
  }
  // follow kinect user in the Z axis
  kinectFollowZ();
  
  //goDownHallway(); // go forward and use front sonars to turn
  //lineFollow(); // follow a black line on a white surface
  
  //debug(); // get all values of all sensors and display at certain speed
} // void loop

/*---------------------------------------------------------------
Gets kinect XYZ values of tracked user from kinect.
----------------------------------------------------------------*/
void getKinectValues()
{
  // check if enough data has been sent from the computer:
  if (Serial2.available()>3) 
  { 
    // Read the first value. beginning of the communication. 
    val = Serial2.read(); 
    // If the value is the event trigger character 'S' 
    if(val == 'S')
    { 
      // read the most recent byte, which is the x-value 
      xVal = Serial2.read();
      // Then read the y-value 
      yVal = Serial2.read();
      // Then read the z-value
      zVal = Serial2.read();
    } // if input is 'S' then read values
  } //if serial read on
} // void getKinectValues()

/*---------------------------------------------------------------
Checks front three sonar sensors. If too close then changes
boolean value to reflect that.
----------------------------------------------------------------*/
void checkFrontSonars()
{
  digitalWrite(trig1, LOW);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig1, LOW);
  // Compute distance
  distance1 = pulseIn(echo1, HIGH); 
  distance1 = distance1/58.0;
  if(distance1 < tooClose)
  {
    isSonarClose1 = true;
  } // if distance < tooClose
  else
  {
    isSonarClose1 = false;
  } // else distance > tooClose
  
  digitalWrite(trig2, LOW);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig2, LOW);
  // Compute distance
  distance2 = pulseIn(echo2, HIGH); 
  distance2 = distance2/58;
  if(distance2 < tooClose)
  {
    isSonarClose2 = true;
  } // if distance < tooClose
  else
  {
    isSonarClose2 = false;
  } // else distance > tooClose
  
  digitalWrite(trig3, LOW);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig3, LOW);
  // Compute distance
  distance3 = pulseIn(echo3, HIGH); 
  distance3 = distance3/58;
  if(distance3 < tooClose)
  {
    isSonarClose3 = true;
  } // if distance < tooClose
  else
  {
    isSonarClose3 = false;
  } // else distance > tooClose
} // void checkFrontSonars

/*---------------------------------------------------------------
Gets back three sonar sensor values. If distance is too close
then changes boolean value.
----------------------------------------------------------------*/
void checkBackSonars()
{
  digitalWrite(trig4, LOW);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig4, HIGH);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig4, LOW);
  // Compute distance
  distance4 = pulseIn(echo4, HIGH); 
  distance4 = distance4/58;
  if(distance4 < tooClose)
  {
    isSonarClose4 = true;
  } // if distance < tooClose
  else
  {
    isSonarClose4 = false;
  } // else distance > tooClose
  
  digitalWrite(trig5, LOW);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig5, HIGH);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig5, LOW);
  // Compute distance
  distance5 = pulseIn(echo5, HIGH); 
  distance5 = distance5/58;
  if(distance5 < tooClose)
  {
    isSonarClose5 = true;
  } // if distance < tooClose
  else
  {
    isSonarClose5 = false;
  } // else distance > tooClose
  
  digitalWrite(trig6, LOW);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig6, HIGH);
  delayMicroseconds(sonarDelay);
  digitalWrite(trig6, LOW);
  // Compute distance
  distance6 = pulseIn(echo6, HIGH); 
  distance6 = distance6/58;
  if(distance6 < tooClose)
  {
    isSonarClose6 = true;
  } // if distance < tooClose
  else
  {
    isSonarClose6 = false;
  } // else distance > tooClose
  
} // void checkBackSonars

/*---------------------------------------------------------------
Method for RC car to go straight and follow front sonar sensors.
----------------------------------------------------------------*/
void goDownHallway()
{ 
  // use front sonars sensors to turn servo   
  sonarDirect();
  
  // if front sensor is too close then stop RC car.
  if(distance2 < tooClose)
  {
    hardStop();
  } // if(distance2 < tooClose)
  else
  {
    hallSensor(downHallwaySpeed);
  } // to make sure the same speed isnt written again
  
} // void goDownHallway

/*
Method to direct RC car to middle of sensors 1 and 3
*/
void sonarDirect()
{   
  if(distance3 - distance1 > sonarWindow && currentServo > servoRight)
  {
    currentServo -= turningAngle;  }
  else if(distance1 - distance3 > sonarWindow && currentServo < servoLeft)
  {
    currentServo += turningAngle;
  }
  else if (abs(distance1 - distance3) < sonarWindow)
  {
    currentServo = servoStraight;
  }
  myServo.write(currentServo);
} // void sonarDirect

/*
Motor follows Z axis of followed user
*/
void kinectFollowZ()
{  
  if(zVal <= distanceFromKinect)
  {
    hardStop();
  }
  else if(zVal > distanceFromKinect)
  {
    currentSpeed = maxSpeed;
    myMotor.write(maxSpeed);
    //hallSensor(kinectSpeed);
  } 
} // void kinectFollowZ

/*
Servo follows the X axis of the followed user.
*/
void kinectFollowX()
{
  // keep track of last currentServo
  float temp = currentServo;
  // need to reverse degree
  currentServo = (abs(xVal-255))*180/255;
  // if currentServo is within range then turn otherwise write last angle
  if(currentServo > minKinect | currentServo < maxKinect)
  {
    myServo.write(currentServo);
  }
  else
  {
    myServo.write(temp);
  }
} // void kinectFollowX

/*
Uses line sensors to follow a black line on a white surface.
*/
void lineFollow()
{
  leftLine = analogRead(leftLineSensor);
  rightLine = analogRead(rightLineSensor);
  if(leftLine > whiteLine && rightLine > whiteLine)
  {
    myServo.write(servoRight);
    delay(500);
    myMotor.write(stopSpeed);
    turnedON = false;
  } // if outside track then turn right and stop after 500ms
  else
  {
    if(leftLine - rightLine > lineWindow && currentServo > servoRight)
    {
      currentServo = currentServo - turningAngle;
    } // turn servo right
    else if(rightLine - leftLine > lineWindow && currentServo < servoLeft)
    {
      currentServo = currentServo + turningAngle;
    }
    else
    {
      if(currentServo < servoStraight && currentServo != servoStraight)
      {
        currentServo = currentServo + turningAngle;
      }
      else if(currentServo != servoStraight)
      {
        currentServo = currentServo - turningAngle;
      }
    } // else straighten wheels and go faster if not maxSpeed
  }
  if(turnedON)
  {
    hallSensor(lineSpeed);
  } // if motor stil on then update speed and turn angle
}

/*
Method to display all of the values from all of the sensors.
*/
void debug()
{
  delay(delayTime);
  Serial.print("delay time: ");
  Serial.println(delayTime);
  Serial.print("current speed: ");
  Serial.print(lastMillis);
  Serial.println(" rads/s");
  Serial.print("current servo: ");
  Serial.println(currentServo);
  Serial.print("sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  Serial.print("sensor 2: ");
  Serial.print(distance2);
  Serial.println(" cm");
  Serial.print("sensor 3: ");
  Serial.print(distance3);
  Serial.println(" cm");
  Serial.print("sensor 4: ");
  Serial.print(distance4);
  Serial.println(" cm");
  Serial.print("sensor 5: ");
  Serial.print(distance5);
  Serial.println(" cm");
  Serial.print("sensor 6: ");
  Serial.print(distance6);
  Serial.println(" cm");
  Serial.print("left line sensor: ");
  Serial.println(leftLine);
  Serial.print("right line sensor: ");
  Serial.println(rightLine);
  Serial.print("kinect X value: ");
  Serial.println(xVal);
  Serial.print("kinect Y value: ");
  Serial.println(yVal);
  Serial.print("kinect Z value: ");
  Serial.println(zVal);
  Serial.println();
} // void debug

/*
Method to stop the car really fast.
*/
void hardStop()
{
    //myMotor.write(fullReverse);
    //delay(200); // hit brakes by going reverse for 0.5 seconds
    myMotor.write(stopSpeed);
} // void hardStop

/*
Method to get the speed of the rc car and change
it based on the input int - speed of the car wanted.
*/
void hallSensor(int speedofCar)
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
  
  //still need to do math here below:
  lastMillis = lastMillis;
  
  if(lastMillis > speedofCar)
  {
    currentSpeed = currentSpeed - 1;
  }
  else
  {
    currentSpeed = currentSpeed + 1;
  }
  myMotor.write(currentSpeed);
} // void hallSensor

