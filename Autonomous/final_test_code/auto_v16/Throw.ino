void load_ball(){

  digitalWrite(sol_pin, HIGH); // Solenoid high = Piston Expand
  digitalWrite(mag_pin, HIGH); // Magnet high = Magnet On
  
  if (debug_enable){
    Serial.println("Arm Opening");
  }
  while (digitalRead(ls_mag) == LOW)        // Arm reached Limit Switch 
  {
    delay(1);
  }
  delay(200);
  
  digitalWrite(sol_pin, LOW);               // Piston moves back
  if (debug_enable){
    Serial.println("Piston Retracting");
  }

  while (digitalRead(ls_sol) == LOW)        // Piston has moved back
  {
    delay(1);
  }
}

void unload_ball(){

  digitalWrite(sol_pin, HIGH); // Solenoid high = Piston Expand
  
  if (debug_enable){
    Serial.println("Piston Opening");
  }

  delay(2000);

  digitalWrite(mag_pin, LOW);
  digitalWrite(sol_pin, LOW);               // Piston moves back
  if (debug_enable){
    Serial.println("Piston Retracting");
  }

  while (digitalRead(ls_sol) == LOW)        // Piston has moved back
  {
    delay(1);
  }
}

void load_gold_ball(){
  
  delay(500);

  digitalWrite(sol_pin_r, HIGH); // Solenoid high = Piston Expand
  digitalWrite(mag_pin_r, HIGH); // Magnet high = Magnet On
  if (debug_enable){
    Serial.println("Arm Opening");
  }
  while (digitalRead(ls_mag_r) == LOW)        // Arm reached Limit Switch 
  {
    delay(1);
  }
  delay(500);
  digitalWrite(sol_pin_r, LOW);               // Piston moves back
  if (debug_enable){
    Serial.println("Piston Retracting");
  }

  while (digitalRead(ls_sol_r) == LOW)        // Piston has moved back
  {
    delay(1);
  }
}

void gold_transfer(){
  delay(2000);
}

void shoot_gold_ball(){
  digitalWrite(mag_pin_r, LOW);               // Shoot
  digitalWrite(LED_transfer, LOW);
  if (debug_enable){
    Serial.println("Arm Release");
  }
}

void shoot_ball(){
  digitalWrite(LED_shoot, HIGH);
  delay(10000);
  digitalWrite(mag_pin, LOW);               // Shoot
  if (debug_enable){
    Serial.println("Arm Release");
  }
  delay(2000);
  digitalWrite(LED_shoot, LOW);
}

boolean check_manual(){
  int counter = 0;
  for (int cnt = 0; cnt < 5; cnt++)
  {
    cm = ultra_dist(US_manual);
    if ((cm > manual_dis_min) && (cm < manual_dis_max))
    {
      counter++;
    }
    else
    {
      counter--;
    }
  }
  if (counter > 0)
  {
   return(true);
   digitalWrite(LED_manual, HIGH);
  }
  else  
  {
   return(false);
   digitalWrite(LED_manual, LOW);
  }
}

void check_manual_reverse(){
  int counter = 0;
  for (int cnt = 0; cnt < 5; cnt++)
  {
    cm = ultra_dist(US_manual_reverse);
    if ((cm > manual_reverse_dis_min) && (cm < manual_reverse_dis_max))
    {
      counter++;
    }
    else
    {
      counter--;
    }
  }
  if (counter > 0)
  {
   manual_reverse_detect = true;   
  }
  else  
  {
   manual_reverse_detect = false; 
  }
}

boolean check_ball_transfer(){
  int counter = 0;
  for (int cnt = 0; cnt < 5; cnt++)
  {
    cm = ultra_dist(US_ball_detect);
    if ((cm > ball_dis_min) && (cm < ball_dis_max))
    {
      counter++;
    }
    else
    {
      counter--;
    }
  }
  if (counter > 0)
  {
    return(true);
    digitalWrite(LED_transfer, HIGH);
  }
  else  
  {
    return(false);
    digitalWrite(LED_transfer, LOW);
  }
}



float ultra_dist(int ultrasonic_Pin)
{
  long a; 
  duration = 0;
  float d = 0.0 ;
  pinModeFast(ultrasonic_Pin, OUTPUT);
  digitalWriteFast(ultrasonic_Pin, HIGH);
  delayMicroseconds(5);
  digitalWriteFast(ultrasonic_Pin, LOW);
  pinModeFast(ultrasonic_Pin, INPUT);
  duration=pulseIn(ultrasonic_Pin,HIGH);
  d = duration / 58.2;
  if (debug_enable){
    Serial.println(d);
  }
  return d;
}
