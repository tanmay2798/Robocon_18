

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
    manual_detect = false;    // Make false when ultrasonic is used
    load_position = false;
    ball_detect = false;
    no_of_throws = 0;
  }
  else if (state == 4){
    shaft = false;
  }
  else if (state == 5)
  {
    shaft = true;
  }
  else if ((state == 6))
  {
    shaft = false;
  }
  else if(state == 7){
    shaft = true;
  }
  else if ((state == 8))
  {
    shaft = true;
    state_9_flag = 0;         // Required in state 3
  }
  else if ((state == 9))
  {
    shaft = false;
    final_align_pos = false;
    throw_done_tz2 = false;
    manual_detect = false;       // Change when manual detect function done. make it false
    ball_detect = false;
    no_of_throws = 0;
  }
  else if (state == 10){
    shaft = false;
  }
  else if (state == 11)
  {
    shaft = true;
  }
  else if (state == 12)
  {
    shaft = true;
    manual_reverse_detect = false;
  }
  else if (state == 13)
  {
    manual_reverse_detect = false;
  }
  else if (state == 14)
  {
    shaft = true;
    junc_counter = 0;
  }
  else if (state == 15)
  {
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
