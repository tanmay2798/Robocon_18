void load_ball(){
  digitalWrite(sol_pin, HIGH); // Solenoid high = Piston Expand
  digitalWrite(mag_pin, HIGH); // Magnet high = Magnet On
  if (debug_enable){
    Serial.println("Arm Opening");
  }
  while (digitalRead(ls_mag) == LOW)        // Arm reached Limit Switch 
  {delay(1);}
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

  delay(1500);

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
  delay(300);
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

void load_initial_gold(){
  digitalWrite(sol_pin_r, HIGH); // Solenoid high = Piston Expand
  digitalWrite(mag_pin_r, HIGH); // Magnet high = Magnet On
}

void gold_transfer(){
  int gold_w = -golden_rack_w;

  golden_rack.motor(2,gold_w/4);
//  while(1){
//    if (debounce(HIGH)){
//      golden_rack.motor(2,0);
//      break;
//    }
//  }
//  delay(400);
  
  golden_rack.motor(2,gold_w);
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
  if (gamble_state && state == 4 && throw_done_tz1){
    int time_stamp = millis();
    while(millis() - time_stamp < 6000){
      check_manual();
      if (manual_detect == false){
        skip_state = true;
        digitalWrite(LED_rs2, HIGH);
        delay(1000);
        return;
      }
    }
  }else{
    if (!skip_state || state != 10)
      delay(2000);
  }
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
    if (digitalRead(IR_manual) == LOW)
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
  for (int cnt = 0; cnt < 5; cnt++)
  {
    if (digitalRead(IR_manual_reverse) == LOW){
      counter++;
    }
    else
    {
      counter--;
    }
    delay(50);
  }
  if (counter > 1)
  {
   manual_reverse_detect = true;  
   digitalWrite(LED_manual, HIGH); 
  }
  else if(counter < -1)
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
  for (int cnt = 0; cnt < 5; cnt++)
  {
    if (digitalRead(IR_ball_detect) == LOW)
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

