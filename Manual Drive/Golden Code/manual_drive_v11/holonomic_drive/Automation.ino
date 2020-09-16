void get_encoder_val(){
  enc_dis = (Enc.read()-dis_origin);
  if (enc_dis < 0){
    enc_dis = -enc_dis;
  }
}
void dis_reset(){
    dis_origin = Enc.read();
}
void automatic_motion(){
  if (automation){
    if (automation_state == 1){
      go_load_from_start();
    }
    else if (automation_state == 2){
      go_tz1_from_load();
    }
  }
}

void go_load_from_start(){
  if((enc_dis) < dis_start_to_load-dis_start_to_load_slow){
    auto_x = max_speed_1;
  }else if (enc_dis >= dis_start_to_load-dis_start_to_load_slow && enc_dis < dis_start_to_load){
    if (auto_x>slow_speed_1){
      auto_x -= 10;
    }
    else{
      auto_x = slow_speed_1;
    }
  }else if (enc_dis >= dis_start_to_load){
    auto_x = 0;
//    automation_state = 0;
    automation = false;
  }
}

void go_tz1_from_load(){
  if(enc_dis < dis_load_to_tz1-dis_load_to_tz1_slow){     // accelerate and then constant speed
    auto_x = -max_speed_2; 
  }else if (enc_dis >= dis_load_to_tz1-dis_load_to_tz1_slow && enc_dis<dis_load_to_tz1 ){   // deaccelerate
    auto_x = -slow_speed_2;
  }else if (enc_dis >= dis_load_to_tz1){
    auto_x = 0;
//    automation_state = 0;
    automation = false;
  }
}

void go_tz1off_from_tz1(){
  if(enc_dis < dis_start_to_load/2){     // accelerate and then constant speed
    auto_x = 100;
  }else if (enc_dis >= dis_start_to_load/2 && enc_dis < dis_start_to_load){   // deaccelerate
    auto_x = 0;
  }else if (enc_dis >= dis_start_to_load){
    auto_x = 0;
//    automation_state = 0;
    automation = false;
  }
}

void go_tz2_from_tz1off(){
  if(enc_dis < dis_start_to_load/2){     // accelerate and then constant speed
    auto_x = 100;
  }else if (enc_dis >= dis_start_to_load/2 && enc_dis < dis_start_to_load){   // deaccelerate
    auto_x = 0;
  }else if (enc_dis >= dis_start_to_load){
    auto_x = 0;
//    automation_state = 0;
    automation = false;
  }
}

void go_load_from_tz2(){
  if(enc_dis < dis_start_to_load/2){     // accelerate and then constant speed
    auto_x = 100;
  }else if (enc_dis >= dis_start_to_load/2 && enc_dis < dis_start_to_load){   // deaccelerate
    auto_x = 0;
  }else if (enc_dis >= dis_start_to_load){
    auto_x = 0;
//    automation_state = 0;
    automation = false;
  }
}

