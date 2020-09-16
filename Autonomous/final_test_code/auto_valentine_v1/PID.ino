void update_PID()
{
  lin_diff = lin_read();
  ang_diff = ang_read();

  lin_differential = (lin_diff - lin_pdiff);
  ang_differential = (ang_diff - ang_pdiff);

  lin_control = Kp1_curr * lin_diff + Kd1_curr * lin_differential;
  ang_control = Kp2_curr * ang_diff + Kd2_curr * ang_differential;
  
  lin_pdiff = lin_diff;
  ang_pdiff = ang_diff;

  pval1 = val1;
  pval2 = val2;
}

void set_K(){
  if (start_read_1 == 0){
    Kp1_curr = 0;
    Kd1_curr = 0;
    Kp2_curr = Kp_s;
    Kd2_curr = Kd_s;
  }else if (start_read_0 == 0){
    Kp1_curr = 0;
    Kd1_curr = 0;
    Kp2_curr = Kp_s;
    Kd2_curr = Kd_s;
  }else{
    if (shaft == true){
      Kp1_curr = -Kp1_x; 
      Kd1_curr = -Kd1_x;
      Kp2_curr = Kp2_x;
      Kd2_curr = Kd2_x;
    }else{
      Kp1_curr = -Kp1_y; 
      Kd1_curr = -Kd1_y;
      Kp2_curr = Kp2_y;
      Kd2_curr = Kd2_y;
    }
  }
}

void small_PID(int LSA)
{
  other_lsa_a = analogRead(LSA);
  other_lsa_error = target - other_lsa_a;
  other_lsa_diff = other_lsa_error - other_lsa_perror;
  other_control = 0.3*other_lsa_error + 0.2*other_lsa_diff;
  other_lsa_perror = other_lsa_error;
}


