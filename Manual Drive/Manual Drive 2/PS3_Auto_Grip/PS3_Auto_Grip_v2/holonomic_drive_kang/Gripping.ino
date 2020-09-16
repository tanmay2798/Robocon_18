void mos_init(){
  pinMode(mospin1,OUTPUT);
  pinMode(mospin2,OUTPUT);
}
void sol_grip(){
  if (grip==true){
    digitalWrite(mospin1,HIGH);
    digitalWrite(mospin2,HIGH);
  }
  else {
    digitalWrite(mospin1,LOW);
    digitalWrite(mospin2,LOW);
  }
}


