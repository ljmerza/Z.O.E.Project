void setup() { 
  Serial.begin(9600); 
  pinMode(2, OUTPUT); 
} 

void loop()
{ 
   int input=Serial.read(); 
   if(input == '1')
   { 
    digitalWrite(2, HIGH); 
   }
   else
   { 
    digitalWrite(2, LOW);
   }
   delay(700);
 }
