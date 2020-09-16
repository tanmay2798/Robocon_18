void load_ball(){
 
  delay(500);

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
    cm = ultrasonic_read();
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

int ultrasonic_read()
{
  pinMode(ultrasonic_pin, OUTPUT);
  digitalWrite(ultrasonic_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonic_pin, HIGH);
  delay(10);
  digitalWrite(ultrasonic_pin, LOW);

  pinMode(ultrasonic_pin, INPUT);
  int duration = pulseIn(ultrasonic_pin, HIGH);
  int cm = microsecondsToCentimeters(duration); 
  return cm;
}

