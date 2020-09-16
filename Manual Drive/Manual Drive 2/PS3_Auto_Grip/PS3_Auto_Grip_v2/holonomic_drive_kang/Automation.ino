void get_encoder_val(){
  enc_dis = abs(Enc.read());
  if (enc_dis > 10000){
    digitalWrite(13, LOW);
  }else{
    digitalWrite(13, HIGH);
  }
}

void automatic_motion(){
  if (automation){
    if (automation_state == 1){
      go_load_from_start();
    }else if (automation_state == 2){
      go_tz1_from_load();
    }
  }
}

void go_load_from_start(){
  if((enc_dis) < dis_load_slow){
    auto_x = 100;
  }else if ((enc_dis) >= dis_load_slow && enc_dis < dis_load){
    auto_x = 30;
  }else if (enc_dis >= dis_load){
    auto_x = 0;
    automation_state = 0;
  }
}

void go_tz1_from_load(){
  if(enc_dis < dis_transfer-dis_stop){     // accelerate and then constant speed
    auto_x = 100; 
  }else if (enc_dis >= dis_transfer-dis_stop && enc_dis<=dis_transfer ){   // deaccelerate
    auto_x = const_speed;
  }else if (enc_dis >= dis_transfer){
    auto_x = 0;
    automation_state = 0;
  }
}

void go_tz1off_from_tz1(){
  if(enc_dis < dis_load/2){     // accelerate and then constant speed
    auto_x = 100;
  }else if (enc_dis >= dis_load/2 && enc_dis < dis_load){   // deaccelerate
    auto_x = 0;
  }else if (enc_dis >= dis_load){
    auto_x = 0;
    automation_state = 0;
  }
}

void go_tz2_from_tz1off(){
  if(enc_dis < dis_load/2){     // accelerate and then constant speed
    auto_x = 100;
  }else if (enc_dis >= dis_load/2 && enc_dis < dis_load){   // deaccelerate
    auto_x = 0;
  }else if (enc_dis >= dis_load){
    auto_x = 0;
    automation_state = 0;
  }
}

void go_load_from_tz2(){
  if(enc_dis < dis_load/2){     // accelerate and then constant speed
    auto_x = 100;
  }else if (enc_dis >= dis_load/2 && enc_dis < dis_load){   // deaccelerate
    auto_x = 0;
  }else if (enc_dis >= dis_load){
    auto_x = 0;
    automation_state = 0;
  }
}

