void mos_init(){
  pinMode(mospin1,OUTPUT);
  pinMode(mospin2,OUTPUT);
}
//void rack_init(){
//  Rack1.start();
//  Rack1.home().wait();
//  Rack1.s(0);
//  Serial.println("Started Rack1");
//}
void sol_grip(){
  if (grip==true){
    digitalWrite(mospin1,HIGH);
    digitalWrite(mospin2,HIGH);
  }
  else {
    digitalWrite(mospin1,LOW);
    digitalWrite(mospin2,LOW);
  }
}
void rack_enc_read(){
  cur_rack_pos = abs(rack_enc.read());
  Serial.println(cur_rack_pos);
}
void rack1_rotate(){
  if (rack_rotate){
    Serial.println("hi");
    while (cur_rack_pos-prev_rack_pos <= rack_angle){
      rack_st.motor(2,rack_w);
      rack_enc_read();    
    }
    rack_st.motor(2,0);
    prev_rack_pos = cur_rack_pos;
    rack_rotate = !rack_rotate;
  }
  else{
     rack_st.motor(0);
  }
}

