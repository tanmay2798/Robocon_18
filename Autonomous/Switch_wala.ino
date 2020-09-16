int sol_pin =30;
int mag_pin =26;
int ls_sol= 29;
int ls_mag =27;
int push_btn= 9;
int i=0;
void setup() {
  // put your setup code here, to run once:
  pinMode(sol_pin,OUTPUT);
  pinMode(ls_sol,INPUT);
  pinMode(mag_pin,OUTPUT);
  pinMode(push_btn,INPUT);
  pinMode(ls_mag,INPUT);
  //pinMode(8,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(sol_pin,LOW); // Solenoid high
  digitalWrite(mag_pin,LOW); // Magnet high 
  Serial.begin(9600);
}
boolean l_switch=true;
int state = 0;
void loop() 
{
  if (state==0)
  {
      if (digitalRead(push_btn)==HIGH){ // push button high
      digitalWrite(13,HIGH);
      digitalWrite(sol_pin,HIGH); // Solenoid high
      digitalWrite(mag_pin,HIGH); // Magnet high
      state = 1;
    }
  }
  else if (state == 1){
    if (digitalRead(ls_mag) == HIGH){ //LS 2 high
      delay(200);
      digitalWrite(sol_pin,LOW);// solenoid low
      state = 2;
    }
  }
  else if (state ==2 ){
    if (digitalRead(ls_sol)==HIGH){ //ls1 high
      delay(2000);
      digitalWrite(mag_pin,LOW);// magnet low
      
      state = 0;
      digitalWrite(8,HIGH);
  delay(100);
  digitalWrite(8,LOW);
    digitalWrite(13,LOW);
    }
  }
  
}

