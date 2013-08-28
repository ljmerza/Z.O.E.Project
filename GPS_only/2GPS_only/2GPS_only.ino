void setup() 
{ 
  Serial.begin(57600);
  Serial1.begin(57600); // for gps interaction
  Serial2.begin(57600); // for google earth interaction
}

void loop()
{  
  // gps read and write info from serial port 1 to serial port 2
  if(Serial1.available())
  {
    Serial.write(Serial1.read());
  }

} // void loop

