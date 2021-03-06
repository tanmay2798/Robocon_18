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
  delay(1000);
  digitalWrite(sol_pin_r, LOW);               // Piston moves back
  if (debug_enable){
    Serial.println("Piston Retracting");
  }

  while (digitalRead(ls_sol_r) == LOW)        // Piston has moved back
  {
    delay(1);
  }
  if (debug_enable){
    Serial.println("Piston is back");
  }
}

void gold_transfer(){
  int gold_w = golden_rack_w;
  if (debug_enable){
    Serial.println("Transferring Gold");
  }
//  golden_rack.motor(1,gold_w);
  delay(200);
  while (digitalRead(limit_golden) == LOW){
    delay(1);
  }
//  golden_rack.motor(1,0);
//  golden_rack.motor(1,gold_w);
  delay(300);
//  golden_rack.motor(1,0);
//  golden_rack.motor(1,-gold_w);
  while(true){
    while (digitalRead(limit_golden) == LOW){
    delay(1);
    }
//    golden_rack.motor(1,0);
    delay(200);
    if (digitalRead(limit_golden) == HIGH){
      break;
    }else{
      gold_w = -gold_w/2;
//      golden_rack.motor(1,gold_w);
      delay(3);
    }
  }
  if (debug_enable){
    Serial.println("Gold Transferred");
  }
}

void shoot_gold_ball(){
  if (debug_enable){
    Serial.println("Shooting Gold");
  }
//  digitalWrite(LED_transfer, HIGH);
  delay(7000);
  digitalWrite(mag_pin_r, LOW);               // Shoot
  delay(2000);
//  digitalWrite(LED_transfer, LOW);
  if (debug_enable){
    Serial.println("Arm Release");
  }
}

void shoot_ball(){
  delay(8000);
  digitalWrite(LED_rs1, HIGH);
  digitalWrite(LED_shoot, HIGH);
  digitalWrite(LED_shoot_2, HIGH);
  digitalWrite(mag_pin, LOW);               // Shoot
  if (debug_enable){
    Serial.println("Arm Release");
  }
  delay(1000);
  digitalWrite(LED_rs1, LOW);
  digitalWrite(LED_shoot, LOW);
  digitalWrite(LED_shoot_2, LOW);
}

void check_manual(){
  if (debug_enable){
    Serial.println("Checking Manual");
  }
  int counter = 0;
  for (int cnt = 0; cnt < 5; cnt++)
  {
//    cm = manual_ultrasonic_read();
//    if ((cm > manual_dis_min) && (cm < manual_dis_max))
//    {
//      counter++;
//    }
//    else
//    {
//      counter--;
//    }u
    if (digitalRead(US_manual) == LOW)
    {
      counter++;
    }
    else
    {
      counter--;
    }
  }
  if (counter > 1)
  {
   manual_detect = true;
   digitalWrite(LED_manual, HIGH);
  }
  else if (counter < -1)
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
  if (!manual_reverse_detect){
    manual_reverse_dis_min = manual_reverse_dis_min_1;
    manual_reverse_dis_max = manual_reverse_dis_max_1;
  }else{
    manual_reverse_dis_min = manual_reverse_dis_min_2;
    manual_reverse_dis_max = manual_reverse_dis_max_2;
  }
  for (int cnt = 0; cnt < 10; cnt++)
  {
    //cm = manual_reverse_ultrasonic_read();
    if (digitalRead(manual_reverse_detect_pin) == LOW)
    {
      counter++;
    }
    else
    {
      counter--;
    }
    delay(50);
  }
  if (counter > 2)
  {
   manual_reverse_detect = true;  
//   digitalWrite(LED_manual, HIGH); 
  }
  else if(counter < -2)
  {
   manual_reverse_detect = false;
//   digitalWrite(LED_manual, LOW); 
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
  if (counter > 1)
  {
    ball_detect = true;
    digitalWrite(LED_transfer, HIGH);
  }
  else if (counter < -1)
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
  if (debug_enable)
  Serial.println(cm);
  return cm;
}

int manual_reverse_ultrasonic_read(){       // NOT USED
  digitalWrite(US_manual_reverse_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual_reverse_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_manual_reverse_trig,LOW);
  long duration = pulseIn(US_manual_reverse_echo,HIGH);
  int dis = duration/58.2;
  if (debug_enable)
  Serial.println(dis);
  return dis;
}

int manual_ultrasonic_read()                // NOT USED
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
  if (debug_enable)
  Serial.println(cm); 
  return cm;
}

void adjust_rack(){
  for (i=0;i<5; i++){
//    golden_rack.motor(1,golden_rack_w*2);
    delay(100);
//    golden_rack.motor(1,-golden_rack_w*2);
    delay(100);
  }
//  golden_rack.motor(1, 0);
}

