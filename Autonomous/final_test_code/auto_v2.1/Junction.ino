void junction()
{
  if ((state != 4) && (state != 6) && (state != 9) && (state != 3) && (state != 10)&& (state != 12)&& (state != 15))
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
      else if (((millis() - state_time_cnt) > 500) && (state == 7))
      {
        junc = true;
      }
    }
    junc_time_cnt = millis();
  }
}

void junc_align(){
  stop_bot();
  if (state == 0){
    junc_move_x(0.5 * base);
    check_line(0);        // loop
      delay(100);
    stop_bot();
    junc_move_x(-0.33 * base);
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
    junc_move_y(-170);
    check_line(2);
    stop_bot();
  }
  else if (state == 6)
  {
    junc_move_x(base/2);
    check_line(0);
    stop_bot();
    junc_move_x(100);
    check_line(1);
    stop_bot();
  }
  else if (state == 7)
  {
    junc_move_y(base/3);
    check_line(0);
//    delay(100);
    stop_bot();
    junc_move_y(-200);
    check_line(1);
    stop_bot();
  }
  else if (state == 11)
  {
    for (i=0; i < 34; i++){
      junc_move_y(5*i);
      
      if ((analogRead(LSA_2) < lsa_star_val)&&(analogRead(LSA_3) < lsa_star_val))
      {
        stop_bot();
        break;
      }
    }
    check_line(2);
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
  }else if(state == 1 || state == 5 || state == 7 || state == 11){
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
    if ((state == 6) || (state == 12)||(state == 15))
    {
      junc = true;
    }
  }
}


