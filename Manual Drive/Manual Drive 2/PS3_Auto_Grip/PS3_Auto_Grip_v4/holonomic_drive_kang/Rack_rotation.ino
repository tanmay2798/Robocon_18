void rack_init(){
  pinMode(rack_1_pin,OUTPUT);
  pinMode(rack_2_pin,OUTPUT);
}
//void rack_init(){
//  Rack1.start();
//  Rack1.home().wait();
//  Rack1.s(0);
//  Serial.println("Started Rack1");
//}
void rack_1_grip(){
  if (rack_1_state==true){
    digitalWrite(rack_1_pin,HIGH);
  }
  else {
    digitalWrite(rack_1_pin,LOW);
  }
}

void rack_2_grip(){
  if (rack_2_state==true){
    digitalWrite(rack_2_pin,HIGH);
  }
  else {
    digitalWrite(rack_2_pin,LOW);
  }
}

void rack_enc_read(){
  cur_rack_pos = abs(rack_enc.read());
  Serial.println(cur_rack_pos);
}
void rack_1_rotate(){
  if (rack_rotate){
    Serial.println("hi");
    while (cur_rack_pos-prev_rack_pos <= rack_angle){
      rack_st.motor(2,rack_w/2);
      rack_enc_read();    
    }
    rack_st.motor(2,0);
    prev_rack_pos = cur_rack_pos;
    rack_rotate = !rack_rotate;
  }
  else{
     rack_st.motor(2,0);
  }
}

