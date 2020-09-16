void junction()
{
  digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    
  if (detect == 2)
  {
    if (shaft == true)
    {
      junc = true;
    }
    else if ((shaft == false ) && (yEnc.read() > 22000))
    {
      junc = true;
      detect = 0;
    }
  }
  else
  {
    detect++;
  }
  delay(100);
}

void junc_move_x()
{
  v1 = base;
  v2 = base / 2;
  v3 = -(base) / 2;
  write_vel();
}
void junc_move_y()
{
  v1 = 0;
  v2 = base;
  v3 = base;
  write_vel();
}
void write_vel()
{
  k3.s(v3);
  k2.s(v2);
  k1.s(v1);
}

void state_change()
{
  k3.s(0);
  k2.s(0);
  k1.s(0);
  if (state == 0)
  {
    baseacc = 4000;
    basedec = 0;
    base = 0;
    acceleration = 40;
    shaft = false;
    valread_y = 0;
    Kp1 = 0, Kd1 = 0;
    Kp2 = 0, Kd2 = 0;
    pval1 = analogRead(pin1);
    pval2 = target;
    val1 = pval1;
    val2 = pval2;
  }
  else if ((state == 1) || (state == 4) || (state == 5))
  {
    baseacc = 2000;
    basedec = 0;
    base = -200;
    shaft = true;
    shaft = true;
    acceleration = 120;
  }
  else if ((state == 2) || (state == 6))
  {
    baseacc = 0;
    basedec = 0;
    base = 0;
    acceleration = 0;
    shaft = false;
    x = millis();
  }
  else if (state == 3)
  {
    acceleration = -120;
    basedec = -2000;
    baseacc = 0;
    detect = 0;
    shaft = true;
    detect = 0;
  }
  state++;
  junc = false;
  delay(500);
}

