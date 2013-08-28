import processing.core.*; 
import processing.data.*; 
import processing.opengl.*; 

import processing.serial.*; 

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

public class prj1 extends PApplet {


Serial myPort;

public void setup()
{
  size(255,255);
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
}

public void draw()
{
  noStroke();
  rect(0,0,width/2,height);
  
  if(mouseX>width/2)
  {
    myPort.write('1');
  } // if mouse on right then write 1 to port
  else
  { 
    myPort.write('0');
  } // else write zero to port
} // draw method


  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "prj1" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
