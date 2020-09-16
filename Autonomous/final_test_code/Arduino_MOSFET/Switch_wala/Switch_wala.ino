int mag_pin_in = 2;
int sol_pin_in = 3;
int mag_pin_g_in = 5;
int sol_pin_g_in = 4;

int mag_pin_out = 9;
int sol_pin_out = 12;
int mag_pin_g_out = 11;
int sol_pin_g_out = 10;
void setup() {
  // put your setup code here, to run once:
  pinMode(mag_pin_in,INPUT);
  pinMode(sol_pin_in,INPUT);
  pinMode(mag_pin_g_in,INPUT);
  pinMode(sol_pin_g_in,INPUT);

  pinMode(mag_pin_out,OUTPUT);
  pinMode(sol_pin_out,OUTPUT);
  pinMode(mag_pin_g_out,OUTPUT);
  pinMode(sol_pin_g_out,OUTPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (digitalRead(mag_pin_in)== HIGH){
    digitalWrite(mag_pin_out,HIGH); 
  }
  else{
    digitalWrite(mag_pin_out, LOW);
  }
  
  if (digitalRead(sol_pin_in)== HIGH){
    digitalWrite(sol_pin_out,HIGH); 
  }
  else{
    digitalWrite(sol_pin_out, LOW);
  }
  
  if (digitalRead(mag_pin_g_in)== HIGH){
    digitalWrite(mag_pin_g_out,HIGH); 
  }
  else{
    digitalWrite(mag_pin_g_out, LOW);
  }
  
  if (digitalRead(sol_pin_g_in)== HIGH){
    digitalWrite(sol_pin_g_out,HIGH); 
  }
  else{
    digitalWrite(sol_pin_g_out, LOW);
  }
  
  
}
