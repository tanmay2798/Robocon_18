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
  pinMode(23, INPUT);
  pinMode(24, INPUT);
  pinMode(27, INPUT);
  pinMode(33, INPUT);
  pinMode(47, INPUT);
  pinMode(49, INPUT);
  pinMode(53, INPUT);
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(34, OUTPUT);
  digitalWrite(sol_pin,LOW);
  digitalWrite(mag_pin,LOW);
  digitalWrite(32,LOW);
  digitalWrite(30,LOW);
  digitalWrite(26,LOW);
  attachInterrupt(digitalPinToInterrupt(49), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(53), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(51), junction_R, RISING);
}

