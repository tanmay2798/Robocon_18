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
  if (debug_enable){
    Serial.println("Arm Release");
  }
}

void shoot_ball(){
  digitalWrite(mag_pin, LOW);               // Shoot
  if (debug_enable){
    Serial.println("Arm Release");
  }
}

void check_manual(){
  int counter;
  for (int cnt = 0; cnt < 5; cnt++)
  { 
    counter = 0;
    cm = manual_ultrasonic_read();
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
   manual_detect = true;   
  }
  else  
  {
   manual_detect = false; 
  }
}

int manual_ultrasonic_read()
{
  pinMode(US_manual_ping, OUTPUT);
  digitalWrite(US_manual_ping, LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual_ping, HIGH);
  delay(5);
  digitalWrite(US_manual_ping, LOW);

  pinMode(US_manual_pulse, INPUT);
  int duration = pulseIn(US_manual_pulse, HIGH);
  int cm = microsecondsToCentimeters(duration); 
  return cm;
}

void check_manual_reverse(){
  int counter;
  for (int cnt = 0; cnt < 5; cnt++)
  { 
    counter = 0;
    cm = manual_reverse_ultrasonic_read();
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

int manual_reverse_ultrasonic_read()
{
  pinMode(US_manual_reverse_ping, OUTPUT);
  digitalWrite(US_manual_reverse_ping, LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual_reverse_ping, HIGH);
  delay(5);
  digitalWrite(US_manual_reverse_ping, LOW);

  pinMode(US_manual_pulse, INPUT);
  int duration = pulseIn(US_manual_reverse_pulse, HIGH);
  int cm = microsecondsToCentimeters(duration); 
  return cm;
}

void check_ball_transfer(){
  int counter;
  for (int cnt = 0; cnt < 5; cnt++)
  { 
    counter = 0;
    cm = ball_detect_ultrasonic_read();
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
    ball_detect = true;   
  }
  else  
  {
    ball_detect = false; 
  }
}

int ball_detect_ultrasonic_read()
{
  pinMode(US_ball_detect_ping, OUTPUT);
  digitalWrite(US_ball_detect_ping, LOW);
  delayMicroseconds(2);
  digitalWrite(US_ball_detect_ping, HIGH);
  delay(5);
  digitalWrite(US_ball_detect_ping, LOW);

  pinMode(US_ball_detect_pulse, INPUT);
  int duration = pulseIn(US_ball_detect_pulse, HIGH);
  int cm = microsecondsToCentimeters(duration); 
  return cm;
}
