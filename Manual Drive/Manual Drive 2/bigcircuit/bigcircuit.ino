int i=0;
void setup() {
  // put your setup code here, to run once:
pinMode(8,OUTPUT);
pinMode(7,INPUT);
pinMode(13,OUTPUT);
pinMode(4,OUTPUT);
Serial.begin(9600);
}
boolean l_switch=true;
void loop() {
  // put your main code here, to run repeatedly:

if(digitalRead(7)==HIGH && l_switch){
  i++;
  Serial.println("change");
  delay(500);
  digitalWritme(7,LOW);
  l_switch = false;
}else  if(digitalRead(7)==LOW){
  l_switch = true;
}
Serial.println("hi");
if(i%3==1){
  Serial.println(i);
  digitalWrite(4,HIGH);
  digitalWrite(8,LOW);
  
}

if(i%3==2){
  Serial.println(i);
  digitalWrite(8,HIGH);
  //delay(500);
  digitalWrite(4,LOW);
}

if(i%3==0){
  Serial.println(i);
  digitalWrite(8,LOW);
  digitalWrite(4,LOW);
}



}
