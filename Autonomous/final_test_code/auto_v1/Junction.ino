void junction()
{
  if ((state != 4) && (state != 5) && (state != 9) && (state != 3))
  {
    if (millis() - junc_time_cnt > 500)
    {
      if (shaft == true)
      {
        junc = true;
      }
      else if ((millis() - state_time_cnt > 3200) && ( state == 1))
      {
        junc = true;
      }
      else if (((millis() - state_time_cnt) > 500) && (state == 6))
      {
        state = 1;
        junc = true;
        dummy_state = true;
      }
    }
    junc_time_cnt = millis();
  }
}

void junc_align(){
  if (state == 0){
    junc_move_x(0.5 * base);
    check_line(0);        // loop
//      delay(100);
    stop_bot();
    junc_move_x(-0.33 * base);
    check_line(1);
    stop_bot();
  }else if (state == 1){
    junc_move_y(base/3);
    check_line(0);
//    delay(100);
    stop_bot();
    junc_move_y(-200);
    check_line(1);
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
  if (state == 0){
    if (check_type == 0){
      while(analogRead(LSA_0) > lsa_star_val)
      {
        delay(1);
      }
    }else{
      while(analogRead(LSA_0) > 700 || analogRead(LSA_0) < 300)
      {
        delay(1);
      }
    }
  }else if(state == 1){
    if (check_type == 0){
      while (analogRead(LSA_3) > lsa_star_val)
      {
        delay(1);
      }
    }else{
      while ((analogRead(LSA_2) > 700) )//| (analogRead(LSA_2) < 300))
      {delay(1);}
    }
  }
}


void junction_R()
{
  if (millis() - junc_time_cnt > 150)
  {
    if ((state == 5) || (state == 9))
    {
      junc = true;
    }
  }
}


