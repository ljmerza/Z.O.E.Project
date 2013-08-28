int c = 57600;

void setup() 
{
  Serial.begin(57600); 
  Serial1.begin(57600);
}
void loop() 
{
  if(Serial1.available())
  {
    Serial.write(Serial1.read());
  }
}
