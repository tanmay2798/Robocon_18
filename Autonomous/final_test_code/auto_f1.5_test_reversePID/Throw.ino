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

boolean debounce(boolean test_type) {
  boolean condition=digitalRead(limit_golden);
  if (condition != prevVal)
  {
    lastDebounceTime = millis();
    while(millis()-lastDebounceTime < debounceTime)
    {
      condition==digitalRead(limit_golden);
      if (condition == prevVal)
      {
        return false;
      }
    }
    prevVal = condition; 
    if (condition == test_type)
    return true;
    else
    return false;
  }
  else 
  {
    return false;
  }
}
//  prevVal = condition;
//  if (millis()-lastDebounceTime > debounceTime && condition==test_type){
//    return true;
//  }
//  else{
//    return false;
//  }
//}

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
  delay(200);
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
  int gold_w = -golden_rack_w;

  golden_rack.motor(2,gold_w);
//  delay(400);
  for(int i = 0; i < throw_cnt_tz3+2; i++)
  {
    while (1)
    {
      if(debounce(LOW))
        break;
    }
    while (1)
    {
      if(debounce(HIGH))
        break;
    }
  }

//  delay(200);
  golden_rack.motor(2,0);
  delay(100);
  if (debug_enable){
  Serial.print(throw_cnt_tz3+1);
  Serial.println("loaded");
  }
//  delay(200);
  if (throw_cnt_tz3 != 3){
    gold_w = -gold_w;
    golden_rack.motor(2,gold_w);
    while (1)
    {
      if(debounce(HIGH))
        break;
    }
    for(int i = 0; i < throw_cnt_tz3+1; i++)
    {
      while (1)
    {
      if(debounce(LOW))
        break;
    }
      while (1)
    {
      if(debounce(HIGH))
        break;
    }
    }
  }
  golden_rack.motor(2,0);
  delay(100);
  if(debounce(LOW))
  {
    gold_w = -gold_w/2;
    golden_rack.motor(2,gold_w);
    
    while(true){
      while (digitalRead(limit_golden)==LOW){
      delay(1);
      }
      golden_rack.motor(2,0);
      delay(100);
      while (digitalRead(limit_golden)==HIGH);
      if (debounce(HIGH)){
        break;
      }else{
        gold_w = -gold_w/2;
        golden_rack.motor(2,gold_w);
      }
    }
  }
//  delay(4000);
}

void shoot_gold_ball(){
  if (debug_enable){
    Serial.println("Shooting Gold");
  }
  delay(2000);
  digitalWrite(LED_junc, HIGH);
  digitalWrite(LED_shoot, HIGH);
  digitalWrite(LED_shoot_2, HIGH);
  digitalWrite(mag_pin_r, LOW);               // Shoot
//  digitalWrite(LED_transfer, LOW);
  if (debug_enable){
    Serial.println("Arm Release");
  }
  delay(1000);
  digitalWrite(LED_junc, LOW);
  digitalWrite(LED_shoot, LOW);
  digitalWrite(LED_shoot_2, LOW);
}

void shoot_ball(){
  delay(2000);
  digitalWrite(LED_junc, HIGH);
  digitalWrite(LED_shoot, HIGH);
  digitalWrite(LED_shoot_2, HIGH);
  digitalWrite(mag_pin, LOW);               // Shoot
  if (debug_enable){
    Serial.println("Arm Release");
  }
  delay(1000);
  digitalWrite(LED_junc, LOW);
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
//  if (!manual_reverse_detect){
//    manual_reverse_dis_min = manual_reverse_dis_min_1;
//    manual_reverse_dis_max = manual_reverse_dis_max_1;
//  }else{
//    manual_reverse_dis_min = manual_reverse_dis_min_2;
//    manual_reverse_dis_max = manual_reverse_dis_max_2;
  for (int cnt = 0; cnt < 10; cnt++)
  {
    //cm = manual_reverse_ultrasonic_read();
    if (digitalRead(manual_reverse_detect_pin) == LOW){
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
   digitalWrite(LED_manual, HIGH); 
  }
  else if(counter < -2)
  {
   manual_reverse_detect = false;
   digitalWrite(LED_manual, LOW); 
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

int ball_detect_ultrasonic_read()
{
  pinMode(US_ball_detect_trig, OUTPUT);
  digitalWrite(US_ball_detect_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(US_ball_detect_trig, HIGH);
  delay(5);
  digitalWrite(US_ball_detect_trig, LOW);

  pinMode(US_ball_detect_echo, INPUT);
  int duration = pulseIn(US_ball_detect_echo, HIGH);
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

void shake_rack(){
  for (i=0;i<5; i++){
    golden_rack.motor(2,golden_rack_w*2);
    delay(100);
    golden_rack.motor(2,-golden_rack_w*2);
    delay(100);
  }
  golden_rack.motor(2, 0);
}

void adjust_rack(){
  int gold_w = golden_rack_w;
  while(true){
    golden_rack.motor(2,gold_w);
    while (digitalRead(limit_golden) == LOW){
    delay(1);
    }
    golden_rack.motor(2,0);
    delay(100);
    if (digitalRead(limit_golden) == HIGH){
      break;
    }else{
      gold_w = -gold_w/2;
      golden_rack.motor(2,gold_w);
    }
  }
}

