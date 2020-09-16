long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}


long absolute(long tmp){
  if (tmp < 0){
    return -tmp;
  }else{
    return tmp;
  }
}

void pin_init(){
  pinMode(LSA_0_i, INPUT);
  pinMode(LSA_2_i, INPUT);
  pinMode(LSA_3_i, INPUT);

  pinMode(LS_4, INPUT);
  pinMode(LS_1, INPUT);
  pinMode(LS_2, INPUT);
  pinMode(LS_3, INPUT);
  
  pinMode(ls_mag, INPUT);
  pinMode(ls_sol, INPUT);
  pinMode(sol_pin, OUTPUT);
  pinMode(mag_pin, OUTPUT);
  
  pinMode(ls_mag_r, INPUT);
  pinMode(ls_sol_r, INPUT);
  pinMode(sol_pin_r, OUTPUT);
  pinMode(mag_pin_r, OUTPUT);
  
  digitalWrite(sol_pin,LOW);
  digitalWrite(mag_pin,LOW);
  digitalWrite(sol_pin_r,LOW);
  digitalWrite(mag_pin_r,LOW);

  attachInterrupt(digitalPinToInterrupt(LSA_0_i), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_2_i), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_3_i), junction_R, RISING);
}

