/*---------------------------------------------------------------
Created by: Leonardo Merza
Version: 2.1
----------------------------------------------------------------*/

/*---------------------------------------------------------------
import serial port and kinect library
----------------------------------------------------------------*/
import processing.serial.*;
import SimpleOpenNI.*;

/*---------------------------------------------------------------
Variables
----------------------------------------------------------------*/
//serial port variable
Serial myPort; 
//kinect variable
SimpleOpenNI  kinect;
//vector of head being tracked
PVector SKEL_HEAD = new PVector();
// images of kinect
PImage kinectDepth;
// threshold of level of confidence
float confidenceLevel = 0.5;
// vector of tracked head for confidence checking
PVector confidenceVector = new PVector();
// the current confidence level that the kinect is tracking
float confidence;

/*---------------------------------------------------------------
Setup method. Enables usb and kinect.  Enables depth sensor
and full user tracking. Creates window.
----------------------------------------------------------------*/
void setup()
{
  //prints out avialable serial ports and their COM number
  println(Serial.list());
  // creates new port at 9600 BAUD
  myPort = new Serial(this, Serial.list()[0], 57600);
  
  // instantiate a new context
  kinect = new SimpleOpenNI(this);
 
  // enable depthMap generation 
  kinect.enableDepth();
  
  // enable skeleton generation for all joints
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_HEAD_HANDS);
 
  // create background color
  background(200,0,0);
  // create draw color
  stroke(255,0,0);
  // create draw thickness
  strokeWeight(1);
  // smooth out drawing
  smooth();
 
  // create a window the size of the depth information
  size(kinect.depthWidth(), kinect.depthHeight()); 
} // void setup()

/*---------------------------------------------------------------
Draw method that loops forever. Updates kinect camera and
draws image.  If Kinect is tracking user, it draws a circle
for a head and writes head XYZ coordinates to serial port.
----------------------------------------------------------------*/
void draw()
{ 
  // update the camera
  kinect.update();
  // get kinect data
  kinectDepth = kinect.depthImage();
  // draw depth image
  image(kinectDepth,0,0); 
 
   // get all user IDs of tracked users
  int[] userList = kinect.getUsers();
  
  // loop through each user to see if tracking
  for(int i=0;i<userList.length;i++)
  {
    if(kinect.isTrackingSkeleton(userList[i]))
    {
      // get condidence level that kinect is tracking head
      confidence = kinect.getJointPositionSkeleton(userList[i],
                          SimpleOpenNI.SKEL_HEAD,confidenceVector);
      // convert head vector to real world coordinates                    
      kinect.convertRealWorldToProjective(confidenceVector, confidenceVector); 
      
      // if confidence of tracking is beyond threshold, then track user
      if(confidence > confidenceLevel)
      {
        // draw a circle for a head 
        circleForAHead(userList[i]);
        //write head coordinates to serial port
        writeHeadCoordinates();
      } //if(confidence > confidenceLevel)
    } //if(kinect.isTrackingSkeleton(userList[i]))
  } //for(int i=0;i<userList.length;i++)
} // void draw()
 
/*---------------------------------------------------------------
Method to draw red circle when head tracking.  Gets joint
postion of head and converts to real world coordinates. Creates
head size of 200 pixels and converts Z coordinate of head to
scalar form.  Finally, it sets fill color to red, and draws a 
circle at the XY xoordinates of head with the diameter a function
of the Z distance. Input is user ID.
----------------------------------------------------------------*/
void circleForAHead(int userID)
{
  // get 3D position of a joint
  kinect.getJointPositionSkeleton(userID,SimpleOpenNI.SKEL_HEAD,SKEL_HEAD);
  // convert real world point to projective space 
  kinect.convertRealWorldToProjective(SKEL_HEAD,SKEL_HEAD);
 
  // a 200 pixel diameter head
  float headsize = 200;
  // create a distance scalar related to the depth (z dimension)
  float distanceScalar = (525/SKEL_HEAD.z);
 
  // set the fill colour to make the circle red
  fill(255,0,0); 
  // draw the circle at the position of the head with the head size scaled by the distance scalar
  ellipse(SKEL_HEAD.x,SKEL_HEAD.y, distanceScalar*headsize,distanceScalar*headsize);
} // void circleForAHead()
 
/*---------------------------------------------------------------
When a new user is detected, print new user and that user's ID.
Start pose detection of that user for skeleton tracking. Input
is the user ID number of the currently detected user.
----------------------------------------------------------------*/
void onNewUser(int userId)
{
  println("New User Detected - userId: " + userId);
 
 // start pose detection named Psi
  kinect.startPoseDetection("Psi",userId);
} // void onNewUser(int userId)
 
/*---------------------------------------------------------------
When a user is lost then print user was lost. Input is user Id
of the user that is lost.
----------------------------------------------------------------*/
void onLostUser(int userId)
{
  println("User Lost - userId: " + userId);
} // void onLostUser(int userId)
 
/*---------------------------------------------------------------
Method starts when user is found. Prints userID and stops pose
detection.  Attempts to calibrate skeleton. Input is a String
of the pose detected and int of the user ID.
----------------------------------------------------------------*/
void onStartPose(String pose,int userId)
{
  println("Start of Pose Detected  - userId: " + userId + ", pose: " + pose);
 
  // stop pose detection
  kinect.stopPoseDetection(userId); 
 
  // start attempting to calibrate the skeleton
  kinect.requestCalibrationSkeleton(userId, true); 
} // void onStartPose(String pose,int userId)

/*---------------------------------------------------------------
When skeleton calibration is started, then print that calibration
has started. Input is userID.
----------------------------------------------------------------*/
void onStartCalibration(int userId)
{
  println("Beginning Calibration - userId: " + userId);
}
 
/*---------------------------------------------------------------
When calibration has ended, print calibration is successful. If
successful, then start tracking skeleton, else start pose
detection again. Input is userID and boolean if calibration
was successful.
----------------------------------------------------------------*/
void onEndCalibration(int userId, boolean successfull)
{
  println("Calibration of userId: " + userId + ", successfull: " + successfull);
 
  if (successfull) 
  { 
    println("  User calibrated");
 
    // begin skeleton tracking
    kinect.startTrackingSkeleton(userId); 
  } // if successful
  else 
  { 
    println("  Failed to calibrate user");
 
    // Start pose detection
    kinect.startPoseDetection("Psi",userId);
  } // else
} // void onEndCalibration(int userId, boolean successfull)

/*---------------------------------------------------------------
Writes the XYZ coordinates of the head tracking to the serial
port. Scales numbers from 0-255.
----------------------------------------------------------------*/
void writeHeadCoordinates()
{ 
  //write to serial for start reading values
  myPort.write('S'); 
  
  myPort.write(int(map(SKEL_HEAD.x,0,kinect.depthWidth(),0,254)));
  print(myPort.read() + " ");
  myPort.write(int(map(SKEL_HEAD.y,0,kinect.depthHeight(),0,254)));
  print(myPort.read() + " ");
  myPort.write(int(map(SKEL_HEAD.z,0,8000,0,254)));
  println(myPort.read() + " ");
} // void writeHeadCoordinates()

