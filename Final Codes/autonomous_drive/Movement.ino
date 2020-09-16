void Kangaroo_Init()
{
  if (kang_enable){
    if (debug_enable)
    Serial.println("Initiating Kangaroo");
    Serial1.begin(115200);
    Serial2.begin(115200);
    k1.start();
    k1.home().wait();
    if (debug_enable)
    Serial.println("k1 start");
    k2.start();
    k2.home().wait();
    if (debug_enable)
    Serial.println("k2 start");
    k3.start();
    k3.home().wait();
    if (debug_enable)
    {
    Serial.println("k3 start");
    Serial.println("Kangaroo Initiated");
    }
  }
}

void write_vel()
{
  if (kang_enable){
    k1.s(v1 * W_1);
    k2.s(v2 * W_2);
    k3.s(v3 * W_3);
  }
}
void stop_bot()
{
  if (kang_enable){
    k3.s(0);
    k2.s(0);
    k1.s(0);
  }
}

void update_vel(){
  if (shaft){
    v1 = -base + L_23 * ang_control;
    v2 = base * 0.5 - L_23 * (-lin_control * 0.866 - ang_control);
    v3 = base * 0.5 + L_23 * (-lin_control * 0.866 + ang_control);
  }else{
    v1 = L_01 * (-lin_control + ang_control);
    v2 = -base * 0.866 - L_01 * (-lin_control * 0.5 - ang_control);
    v3 = base * 0.866 - L_01 * (-lin_control * 0.5 - ang_control);
  }
}

void rotate_bot(){
  v1 = omega;
  v2 = omega;
  v3 = omega;
  write_vel();
  delay(1200);
  int count = 0;
  while (1)
  {
    if (analogRead(LSA_2) < 700 || analogRead(LSA_3) < 700){
      count++;
    }else{
      count = 0;
    }
    if (count > 2){
      break;
    }
    update_PID();
    delay(1);
  }
  stop_bot();
}

