import processing.core.*; 
import processing.data.*; 
import processing.opengl.*; 

import processing.serial.*; 
import SimpleOpenNI.*; 

import java.applet.*; 
import java.awt.Dimension; 
import java.awt.Frame; 
import java.awt.event.MouseEvent; 
import java.awt.event.KeyEvent; 
import java.awt.event.FocusEvent; 
import java.awt.Image; 
import java.io.*; 
import java.net.*; 
import java.text.*; 
import java.util.*; 
import java.util.zip.*; 
import java.util.regex.*; 

public class processing extends PApplet {

/*
Created by: Leonardo Merza
Version: 0.7
*/




SimpleOpenNI kinect;
Serial myPort;

//main variables
int closestValue; 
int closestX;
int closestY;

//debugging variables
int closex, closey;
int maxx=630, maxy=478, minx=1, miny=1;

public void setup()
{
  myPort = new Serial(this, Serial.list()[0], 9600);
  
  size(640, 480);
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableIR();
} // void setup

public void draw()
{
  closestValue = 8000; 
  kinect.update();
  // get the depth array from the kinect
  int[] depthValues = kinect.depthMap();
  // for each row in the depth image
  for(int y = 0; y < 480; y++)
  { 
    // look at each pixel in the row
    for(int x = 0; x < 640; x++)
    { 
      // pull out the corresponding value from the depth array
      int i = x + y * 640; 
      int currentDepthValue = depthValues[i];
      // if that pixel is the closest one we've seen so far
      if(currentDepthValue > 0 && currentDepthValue < closestValue)
      { 
        // save its value
        closestValue = currentDepthValue;
        // and save its position (both X and Y coordinates)
        closestX = x;
        closestY = y;
      } // if statement
    } // for loop y value
  } // for loop x value
  
  //draw the depth image on the screen
  image(kinect.irImage(),0,0);
  // draw a red circle over it
  fill(255,0,0);
  ellipse(closestX, closestY, 10, 10);

  //debugging find max/min coordinates
  printMaxMin();
  
  // write start recording byte to serial port 
  myPort.write('S'); 
  // currently set to 0-255 for LEDs 
  myPort.write(closestX);
  myPort.write(closestY);
} // void draw

//debeg method print max/min values of x/y coordinates
public void printMaxMin()
{
  closex = closestX;
  closey = closestY;
  
  if(closex > maxx)
  {
    maxx = closex;
    print("max x:");
    println(closestX);
  }
  
    if(closex < minx)
  {
    minx = closex;
    print("min x:");
    println(closestX);
  }
  
  if(closey > maxy)
  {
    maxy = closey;
    print("max y:");
    println(closestY);
  }
  
  if(closey < miny)
  {
    miny = closex;
    print("min y:");
    println(closestY);
  }
} // void printMaxMin
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--full-screen", "--bgcolor=#666666", "--stop-color=#cccccc", "processing" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
