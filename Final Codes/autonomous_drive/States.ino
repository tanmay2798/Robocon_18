void state_change()
{
  stop_bot();
//  delay(100);
  ///////////////////////////////////////////////////////////////////////
  if (state == 0)
  {
    shaft = false;
    start_read_1 = 0;       // 
    if (retry_state == 0){
      state = 1;
    }else{
      state = 21;
    }
  }
  else if ((state == 1))
  {
    shaft = true;
    state = 2;
  }
  else if ((state == 2))
  {
    shaft = true;
    state_3_flag = 0;         // Required in state 3
    state = 3;
  }
  else if ((state == 3))
  {
    shaft = false;
    final_align_pos = false;
    throw_done_tz1 = false;
    manual_detect = false;    // Make false when ultrasonic is used
    load_position = false;
    ball_detect = false;
    throw_cnt_tz1 = 0;
    state = 4;
  }
  else if (state == 4){
    shaft = false;
    state = 5;
  }
  else if (state == 5)
  {
    shaft = false;
    state = 7;
  }
  else if ((state == 6))
  {
    shaft = false;
    state = 7;
  }
  else if(state == 7){
    shaft = true;
    state = 8;
  }
  else if ((state == 8))
  {
    shaft = true;
    state_9_flag = 0;         // Required in state 9
    state = 9;
  }
  else if ((state == 9))
  {
    shaft = false;
    final_align_pos = false;
    throw_done_tz2 = false;
    manual_detect = false;       // Change when manual detect function done. make it false
    ball_detect = false;
    throw_cnt_tz2 = 0;
    state = 10;
  }
  else if (state == 10){
    shaft = false;
    state = 11;
  }
  else if (state == 11)
  {
    shaft = true;
    state = 12;
  }
  else if (state == 12)
  {
    shaft = true;
    manual_reverse_detect = false;
    state = 13;
  }
  else if (state == 13)
  {
    shaft = true;
    align_state = true;
    manual_reverse_detect = false;
    state = 14;
  }
  else if (state == 14)
  {
    shaft = true;
    align_state = false;
    junc_counter = 0;
    state = 15;
  }
  else if (state == 15)
  {
    shaft = false;
    state = 16;
    throw_cnt_tz3 = 0;
    prevVal=digitalRead(limit_golden);
  }
  else if (state == 16)
  {
    shaft = false;
    state = 17;
    throw_done_tz3 = true;
  }
  else if (state == 17)
  {
    shaft = true;
    state = 18;
  }
  else if (state == 18)
  {
    shaft = true;
    state = 19;
  }
  else if (state == 19)
  {
    shaft = false;
    state = 9;
  }
  else if (state == 21)
  {
    shaft = true;
    if (retry_state == 1){
      state = 8;
    }else{
      state = 32;
    }
  }
  else if (state == 30)
  {
    shaft = false;
    start_read_0 = 0;       // Check
    junc_counter = 0;
    state = 31;
  }
  else if (state == 31)
  {
    shaft = true;
    if (retry_state == 2){
      state = 32;
    }else if(retry_state == 3){
      state = 15;
    }
  }
  else if (state == 32){
    shaft = true;
    if (retry_state == 2){
      state = 14;
    }else{
      state = 15;
    }
  }
  //////////////////////////////////////////////////////////////////////////  
  reset_LSA();
  set_K();
  if (junc == false){
  junc_pos_x = xEnc.read();
  }
  max_base = max_base_val[state];
  min_base = min_base_val[state];
  base = 0;
  acceleration = acceleration_val[state];
  digitalWrite(LED_junc, LOW);
  junc = false;
  junc_counter = 0;
  state_time_cnt = millis();
}

void set_retry_state(){                     // retry
  time_2_slow[21] = 3500;
  min_base_val[21] = 0;
  max_base_val[21] = 2000;
  acceleration_val[21] = 4;
  deacceleration_val[21] = -14;

  min_base_val[30] = -200;
  max_base_val[30] = 0;
  acceleration_val[30] = -3;

  time_2_slow[31] = 3500;
  min_base_val[31] = -2000;
  max_base_val[31] = 0;
  acceleration_val[31] = -4;
  deacceleration_val[31] = 14;
  
  time_2_slow[32] = 1000;
  min_base_val[32] = 200;
  max_base_val[32] = 600;
  acceleration_val[32] = 4;
  deacceleration_val[32] = -14;
  
  // Uncomment after retry boards installed
  boolean r_b1 = digitalRead(retry_b1);
  boolean r_b2 = digitalRead(retry_b2);
  boolean r_b3 = digitalRead(retry_b3);
  if (debug_enable){
  Serial.print("Retry buttons: ");
  Serial.print(r_b1);
  Serial.print(r_b2);
  Serial.println(r_b3);
  }
  
  /// FFF: Normal run, TFF: To TZ2, FTF: To near TZ2 and wait for gold rack transfer, FFT: To TZ3 directly
  if (!r_b1 && !r_b2 && !r_b3){           // all off: || Full Run
    retry_state = 0;
  }else if (r_b1 && !r_b2 && !r_b3){      // B1: On B2,B3: Off || To TZ2
    retry_state = 1;
  }else if (!r_b1 && r_b2 && !r_b3){      // B2:On  B1,B3: Off || To TZ2 for golden rack loading
    state = 0;
    retry_state = 2;
  }else if (!r_b1 && !r_b2 && r_b3){      // B3:ON  B1,B2:OFF  || To TZ3 for shooting
    state = 0;
    retry_state = 3;
  }else if (!r_b1 && r_b2 && r_b3){      // B1:OFF  B3,B2:ON  || TZ1 shooting practice
    state = 91;
    retry_state = 91;
  }else if (r_b1 && r_b2 && !r_b3){      // B3:OFF  B1,B2:ON  || TZ3 shooting practice
    state = 15;
    state_change();
    retry_state = 93;
  }else if (r_b1 && r_b2 && r_b3){      // B3,B1,B2:ON  || Debug State
    state = 99;
    retry_state = 99;
  }else if (r_b1 && !r_b2 && r_b3){      // 101  || Gamble
    gamble_state = true;
  }
}

