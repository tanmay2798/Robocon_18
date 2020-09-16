float target1=0.0,target2=0.0,target3=0.0;
float d1=0.0,d2=0.0,d3=0.0;
float p1=0.0,p2=0.0,p3=0.0;
float s1,s2,s3,vel;
float r1,r2,r3;
void back_Kangaroo_init() {
  K1.start();
  //Serial.println("Started K1");
  
  K1.home().wait();
  K1.s(0);
  //Serial.println("Back channel 1 started");
  K2.start();
  K2.home().wait();
  K2.s(0);
  //Serial.println("Back Channel 2 started");
}
void front_kangaroo_init() {
  M1.start();
  M1.home().wait();
  M1.s(0);
  //Serial.println("Front Channel started");
}
float drive_kang(float target,float perdiff,float curvel){
  if (perdiff>=0.0){
    if (perdiff==1){
      vel=0.01*drive_kang_ramp;
    }
    else{
      vel=(1-perdiff)*drive_kang_ramp;
    }
    /*if (perdiff==1){
      vel+=0.01*target;
    }
    else if (0<=perdiff<=0.1){
      vel=target;
    }
    else if(perdiff<=0.3){
      vel+=target*pow((1-perdiff),0.5);
    }
    else if (perdiff<=0.5){
      vel+=target*pow((1-perdiff),1);
    }
    else if (perdiff<=0.8){
      vel+=target*pow((1-perdiff),1.5);
    }
    else if (perdiff<=0.9){
     vel+=target*pow((1-perdiff),2);
    }
    else if (perdiff<1){
      vel+=target*pow(1-perdiff,2.5);
    }
    vel=constrain(vel,0,target);*/
  }
  else{
    if (perdiff==-1){
      vel=0.01*drive_kang_ramp;
    }
    else{
      vel=(1+perdiff)*drive_kang_ramp;
    }
    /*if (perdiff==-1){
      vel+=0.005*target;
    }
    else if (-0.1<=perdiff<=0){
      vel=target;
    }
    else if(-0.3<=perdiff){
      vel+=target*pow((1+perdiff),1);
    }
    else if (-0.5<=perdiff){
      vel+=target*pow((1+perdiff),1.5);
    }
    else if (-0.8<=perdiff){
      vel+=target*pow((1+perdiff),2);
    }
    else if (-0.9<=perdiff){
     vel+=target*pow((1+perdiff),2.5);
    }
    else if (-1<perdiff){
      vel+=target*pow(1+perdiff,3);
    }
    vel=constrain(vel,target,0);*/
  }
  return vel;
}

void drive_kangaroo_update(float motor1, float motor2, float motor3, boolean ramp) {
  /*target1=motor1*speed_factor;
  target2=motor2*speed_factor;
  target3=motor3*speed_factor;
  s1=K1.getS().value();s2=K2.getS().value();s3=M1.getS().value();
  Serial.print(s1);
  d1=s1-target1;d2=s2-target2;d3=s3-target3;
  if (target1!=0)p1=d1/target1;
  else p1=0.0;
  if (target2!=0)p2=d2/target2;
  else p2=0.0;
  if (target3!=0)p3=d3/target3;
  else p3=0.0;
  r1=drive_kang(target1,p1,s1);
  r2=drive_kang(target2,p2,s2);
  r3=drive_kang(target3,p3,s3);
  K1.s(target1,r1);
  K2.s(target2,r2);
  M1.s(target3,r3);*/
  if (ramp) {
    K1.s(motor1 * speed_factor, drive_kang_ramp);
    K2.s(motor2 * speed_factor, drive_kang_ramp);
    M1.s(motor3 * speed_factor, drive_kang_ramp);
  } else {
    K1.s(motor1 * speed_factor);
    K2.s(motor2 * speed_factor);
    M1.s(motor3 * speed_factor);
  }
}
