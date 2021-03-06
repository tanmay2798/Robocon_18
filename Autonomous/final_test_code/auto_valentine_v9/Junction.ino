void junction()
{
  if ((state != 4) && (state != 6) && (state != 9) && (state != 3) && (state != 10)&& (state != 12)&& (state != 15) && state!= 30 && state != 31 && state != 32)
  {
    if (millis() - junc_time_cnt > 500)
    {
      if (shaft == true)
      {
        junc = true;
        digitalWrite(LED_junc, HIGH);     
        junc_pos_x = xEnc.read();
        junc_pos_y = yEnc.read();
      }
      else if ((millis() - state_time_cnt > 3200) && ( state == 1 || state == 21))
      {
        junc = true;    
        digitalWrite(LED_junc, HIGH);   
        junc_pos_x = xEnc.read();
        junc_pos_y = yEnc.read();
      }
      else if (((millis() - state_time_cnt) > 500) && (state == 7))
      {
        junc = true;   
        digitalWrite(LED_junc, HIGH);    
        junc_pos_x = xEnc.read();
        junc_pos_y = yEnc.read();
      }
    }
    junc_time_cnt = millis();
  }
}

void junc_align(){
  stop_bot();
  if (state == 0){
    junc_move_x(0.8 * base);
    check_line(0);        // loop
      delay(100);
    stop_bot();
    junc_move_x(-0.45 * base);
    check_line(1);
    stop_bot();
  }
  else if (state == 1)
  {
    junc_move_y(base/3);
    check_line(0);
//    delay(100);
    stop_bot();
    junc_move_y(-200);
    check_line(1);
    stop_bot();
  }
  else if (state == 5){
    for(i = 0; i<200; i++){
      junc_move_y(-i*2);
    }
    check_line(1);
    stop_bot();
  }
  else if (state == 6)
  {
    junc_move_x(base);
    check_line(0);
    stop_bot();
    junc_move_x(100);
    check_line(1);
    stop_bot();
  }
  else if (state == 7)
  {
    junc_move_y(0.6 * base);
    check_line(0);
//    delay(100);
    stop_bot();
    junc_move_y(-200);
    check_line(1);
    stop_bot();
  }
  else if (state == 11)
  {
    for(i = 0; i<200; i++){
    junc_move_y(i*2);
    }
    check_line(1);
    stop_bot();
  }
  else if (state == 15)
  {
    junc_move_x(-150);
    check_line(0);        // loop
    stop_bot();
    check_line(2);
    stop_bot();
  }
  else if (state == 17){
    junc_move_y(-400);
    check_line(2);
    stop_bot();
  }
  else if (state == 21)
  {
    junc_move_y(base/3);
    check_line(0);
    stop_bot();
    junc_move_y(-200);
    check_line(1);
    stop_bot();
  }
  else if (state == 30){
    junc_move_x(0.5 * base);
    check_line(0);        // loop
    delay(100);
    stop_bot();
    junc_move_x(-0.33 * base);
    check_line(1);
    stop_bot();
  }
  else if (state == 31)
  {
    junc_move_y(base/3);
    check_line(2);
//    delay(100);
    stop_bot();
    junc_move_y(200);
    check_line(2);
    stop_bot();
  }
}

void junc_move_x(int l_base)
{
     v1 = -l_base;
    v2 = l_base * 0.5;
    v3 = l_base * 0.5;
  write_vel();
}

void junc_move_y(int l_base)
{
  v1 = 0;
  v2 = -l_base;
  v3 = l_base;
  write_vel();
}

void check_line(int check_type){
  if (state == 0 || state == 15 || state == 16){
    if (check_type == 0){
      while(analogRead(LSA_0) > lsa_star_val || analogRead(LSA_0) < 10)
      {
        delay(1);
      }
    }else if (check_type == 1){
      while(analogRead(LSA_0) > lsa_star_val)// || analogRead(LSA_0) < 500)
      {
        delay(1);
      }
    }
  }
  else if (state == 30)
  {
    if (check_type == 0){
      while(analogRead(LSA_1) > lsa_star_val || analogRead(LSA_1) < 10)
      {
        delay(1);
      }
    }else if (check_type == 1){
      while(analogRead(LSA_1) > lsa_star_val)// || analogRead(LSA_0) < 500)
      {
        delay(1);
      }
    }
  }else if(state == 1 || state == 5 || state == 7 || state == 11 || state == 17 || state == 21 || state == 31){
    if (check_type == 0){
      while (analogRead(LSA_3) > lsa_star_val)
      {
        delay(1);
      }
    }else if (check_type == 1){
      while ((analogRead(LSA_3) > lsa_star_val))//||(analogRead(LSA_2) < 300))
      {delay(1);}
    }
    else{
      while ((analogRead(LSA_2) > lsa_star_val)||(analogRead(LSA_3) > lsa_star_val))
      {delay(1);}
    }
  }else if (state == 6){
    if (check_type == 0){
      while(analogRead(LSA_0) > lsa_star_val)
      {
        delay(1);
      }
    }else{
      while(analogRead(LSA_0) > lsa_star_val)
      {
        delay(1);
      }
    }
  }
}


void junction_R()
{
  if (millis() - junc_time_cnt > 250)
  {
    if ((state == 6) || (state == 12)||(state == 15) || state == 30 || state == 31)
    {
      junc = true;      
      digitalWrite(LED_junc, HIGH); 
      junc_pos_x = xEnc.read();
      junc_pos_y = yEnc.read();
    }
  }
}


