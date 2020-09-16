const int pingPin = 12;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop() {
 digitalWrite(13,HIGH);
  long duration, inches, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin,LOW);
  delay(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  
  Serial.println(cm);

/*if(cm<240)
{
  digitalWrite(13,HIGH);
}

else
{
  digitalWrite(13,LOW);
}*/
  
 
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
