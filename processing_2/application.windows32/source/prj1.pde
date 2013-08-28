import processing.serial.*;
Serial myPort;

void setup()
{
  size(255,255);
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
}

void draw()
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


