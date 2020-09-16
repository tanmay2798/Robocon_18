long absolute(long tmp){
  if (tmp < 0){
    return -tmp;
  }else{
    return tmp;
  }
}

void pin_init(){
  int  temp_power_supply = A9;
  pinMode(13, OUTPUT);
  pinMode(temp_power_supply, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);

  digitalWrite(A4, LOW);
  digitalWrite(A5, HIGH);
  digitalWrite(A6, HIGH);
  digitalWrite(A7, HIGH);
  digitalWrite(A8, HIGH);
  digitalWrite(A9, HIGH);
  digitalWrite(A10, HIGH);
  digitalWrite(A11, HIGH);

  pinMode(retry_b1, INPUT);
  pinMode(retry_b2, INPUT);
  pinMode(retry_b3, INPUT);
  
  pinMode(LSA_0_i, INPUT);
  pinMode(LSA_1_i, INPUT);
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

  pinMode(LED_junc, OUTPUT);
  pinMode(LED_manual, OUTPUT);
  pinMode(LED_transfer, OUTPUT);
  pinMode(LED_shoot, OUTPUT);
  pinMode(LED_shoot_2, OUTPUT);

  digitalWrite(temp_power_supply, HIGH);

  digitalWrite(LED_manual,LOW);
  digitalWrite(LED_transfer,LOW);
  digitalWrite(LED_shoot,LOW);
  
  digitalWrite(sol_pin,LOW);
  digitalWrite(mag_pin,LOW);
  digitalWrite(sol_pin_r,LOW);
  digitalWrite(mag_pin_r,LOW);

  pinMode(IR_ball_detect,INPUT);
  pinMode(IR_manual,INPUT);
  pinMode(IR_manual_reverse, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LSA_0_i), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_2_i), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_3_i), junction_R, RISING);
  attachInterrupt(digitalPinToInterrupt(LSA_1_i), junction_R, RISING);
}

