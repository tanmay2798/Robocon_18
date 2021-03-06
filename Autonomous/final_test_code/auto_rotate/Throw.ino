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
  delay(1000);
  digitalWrite(LED_shoot, LOW);
}

void check_manual(){
  if (debug_enable){
    Serial.println("Checking Manual");
  }
  int counter = 0;
  for (int cnt = 0; cnt < 5; cnt++)
  {
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
   digitalWrite(LED_manual, HIGH);
  }
  else  
  {
   manual_detect = false;
   digitalWrite(LED_manual, LOW);
  }
  if (debug_enable){
    Serial.print("Manual = ");
    Serial.println(manual_detect);
  }
}

void check_manual_reverse(){
  int counter = 0;
  for (int cnt = 0; cnt < 5; cnt++)
  {
    cm = manual_ultrasonic_read();
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

void check_ball_transfer(){
  if (debug_enable){
    Serial.println("Checking Ball Transfer");
  }
  int counter = 0;
  for (int cnt = 0; cnt < 20; cnt++)
  {
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
    digitalWrite(LED_transfer, HIGH);
  }
  else  
  {
    ball_detect = false;
    digitalWrite(LED_transfer, LOW);
  }
  if (debug_enable){
    Serial.print("Ball Transfer = ");
    Serial.println(ball_detect);
  }
}



//float ultra_dist(int ultrasonic_Pin)
//{
//  long a; 
//  duration = 0;
//  float d = 0.0 ;
//  pinModeFast(ultrasonic_Pin, OUTPUT);
//  delayMicroseconds(2);
//  digitalWriteFast(ultrasonic_Pin, HIGH);
//  delayMicroseconds(5);
//  digitalWriteFast(ultrasonic_Pin, LOW);
//  delayMicroseconds(2);
//  pinModeFast(ultrasonic_Pin, INPUT);
//  delayMicroseconds(2);
//  while (!digitalReadFast(ultrasonic_Pin));
//  a = micros();
//  while (digitalReadFast(ultrasonic_Pin) && (micros() - a) < 4500); // 5 millisecond timeout
//  duration = micros() - a;
//  d = duration / 58.2;
//  delayMicroseconds(200); // 16/4 -- for four sensors
//  Serial.println(d);
//  return d;
//}

int ball_detect_ultrasonic_read()
{
  pinMode(US_ball_detect, OUTPUT);
  digitalWrite(US_ball_detect, LOW);
  delayMicroseconds(2);
  digitalWrite(US_ball_detect, HIGH);
  delay(5);
  digitalWrite(US_ball_detect, LOW);

  pinMode(US_ball_detect, INPUT);
  int duration = pulseIn(US_ball_detect, HIGH);
  int cm = microsecondsToCentimeters(duration); 
  Serial.println(cm);
  return cm;
}

int manual_ultrasonic_read()
{
  pinMode(US_manual, OUTPUT);
  digitalWrite(US_manual, LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual, HIGH);
  delay(5);
  digitalWrite(US_manual, LOW);

  pinMode(US_manual, INPUT);
  int duration = pulseIn(US_manual, HIGH);
  int cm = microsecondsToCentimeters(duration);
  Serial.println(cm); 
  return cm;
}
