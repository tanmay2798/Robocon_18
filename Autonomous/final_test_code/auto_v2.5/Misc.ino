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
  int  temp_ground = 45;
  pinMode(13, OUTPUT);
  pinMode(temp_ground, OUTPUT);
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

  pinMode(LED_manual, OUTPUT);
  pinMode(LED_transfer, OUTPUT);
  pinMode(LED_shoot, OUTPUT);

  digitalWrite(temp_ground, LOW);

  digitalWrite(LED_manual,LOW);
  digitalWrite(LED_transfer,LOW);
  digitalWrite(LED_shoot,LOW);
  
  digitalWrite(sol_pin,LOW);
  digitalWrite(mag_pin,LOW);
  digitalWrite(sol_pin_r,LOW);
  digitalWrite(mag_pin_r,LOW);

  pinMode(US_manual_reverse_trig,OUTPUT);
  pinMode(US_manual_reverse_echo,INPUT);
  pinMode(manual_reverse_detect_pin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LSA_0_i), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_2_i), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_3_i), junction_R, RISING);
}

