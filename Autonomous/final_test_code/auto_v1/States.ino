

void state_change()
{
  stop_bot();
  ///////////////////////////////////////////////////////////////////////
  if (state == 0)
  {
    shaft = false;
    Kp1_curr = 0, Kd1_curr = 0;
    Kp2_curr = Kp_s, Kd2_curr = Kd_s;
    start_read_1 = 0;       // Check
  }
  else if ((state == 1))
  {
    shaft = true;
  }
  else if ((state == 2))
  {
    shaft = true;
    state_3_flag = 0;         // Required in state 3
  }
  else if ((state == 3))
  {
    shaft = false;
    final_align_pos = false;
    throw_done_tz1 = false;
    manual_detect = false;
  }
  else if (state == 4){
    shaft = false;
  }
  else if (state == 5)
  {
    shaft = true;
    acceleration = -40;
    min_base = -1450;
    max_base = -300;
    base = 0;
    // x = millis();
  }
  else if ((state == 5))
  {
    max_base = 840;
    min_base = 0;
    base = 0;
    acceleration = 50;
//    // x = millis();
    shaft = false;
  }
  if (state == 6)
  {
    acceleration = 60;
    max_base = 420;
    min_base = 0;
    base = 0;
  }
  if (state == 7)
  {
    acceleration = 0;
    base = 0;
    min_base = 0;
    max_base = 0;
    manual_detect = false;
  }
  if (state == 8)
  {
    acceleration = -30;
    base = 0;
    min_base = -840;
    max_base = 0;
    shaft = true;
    junc_R = 0;
  }
  if (state == 9)
  {
    acceleration = 0;
    base = 0;
    min_base = 0;
    max_base = 0;
    shaft = true;
  }
  //////////////////////////////////////////////////////////////////////////  
  state++;
  reset_LSA();
  set_K();
  max_base = max_base_val[state];
  min_base = min_base_val[state];
  base = 0;
  acceleration = acceleration_val[state];
  junc = false;
  junc_pos_x = xEnc.read();
  junc_pos_y = yEnc.read();
  state_time_cnt = millis();
}
