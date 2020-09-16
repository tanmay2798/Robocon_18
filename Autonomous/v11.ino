#include <Kangaroo.h>
#include <Sabertooth.h>
#include <Encoder.h>

float target = 475, ang_control = 0, control = 0, v1 = 0, v2 = 0, v3 = 0;
float t = 0, x = 0, cnt = 0, flag = 0, state = 0, valread_y = 0, junc_R = 0, detect = 2, junc_t = 0, neg_vel = 0;
long int posx = 0, velx  = 0, posy = 0, vely  = 0, prev = 0, junc_pos = 0, x_finalpos = 0;
int val = 0, val1 = 0, val2 = 0, pval1 = 0, pval2 = 0, ang = 0, i = 0;
bool junc = false, shaft = true, final_pos = false, dummy_state = false;
float Kp1 = -0.225, Kd1 = -0.65;      //float Kp1=-0.225,Kd1=-0.65;
float Kp2 = 0.4, Kd2 = 0.3;           //float Kp2=0.4,Kd2=0.3;
float difference = 0, pdifference = 0, differential = 0;
float ang_difference = 0, ang_pdifference = 0, ang_differential = 0;
int pin1 = A0, pin2 = A1, pin3 = A2, pin4 = A3;
long basedec = 0, baseacc = 0, acceleration = 0, deceleration = 0, base = 0;
KangarooSerial K1(Serial3);
KangarooSerial K2(Serial2);
KangarooChannel k1(K1, '1');
KangarooChannel k2(K2, '2');
KangarooChannel k3(K1, '2');
Encoder yEnc(2, 3);
Encoder xEnc(18, 19);

void junc_move_x();
void junc_move_y();
void state_change();
void junction();
void junction_R();
void setup()
{
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  attachInterrupt(digitalPinToInterrupt(20), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(21), junction_R, RISING);

  Kangaroo_Init();

  pinMode(13, OUTPUT);

  pval1 = analogRead(pin4);
  pval2 = analogRead(pin3);

  basedec = 0;
  baseacc = 3500;
  acceleration = 70;
  junc = false;    // state 0
  shaft = true;    // state 0
  detect = 2;
  delay(100);
}

void loop()
{
  base = constrain(base + acceleration, basedec, baseacc);

  difference = LSAread();
  ang_difference = angRead();

  if (millis() != t)
  {
    differential = (pdifference - difference) / (millis() - t);
    ang_differential = (ang_difference - ang_pdifference) / (millis() - t);
    t = millis();
  }

  control = Kp1 * difference + Kd1 * differential;
  ang_control = Kp2 * ang_difference + Kd2 * ang_differential;
  pdifference = difference;
  ang_pdifference = ang_difference;

  if (junc == false)
  {
    if ((state == 0) || (state == 2) || (state == 5))
    {
      v1 = base + ang_control;
      v2 = (base / 2) - ang_control + control;
      v3 = (-base / 2) + ang_control + control;

      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
      if (state == 2)
      {
        if ((abs(xEnc.read() - junc_pos)) >14000)
        {
          v1 = 0;
          v2 = 0;
          v3 = 0;
          write_vel();
          state_change();
        }
        else if (abs(xEnc.read() - junc_pos) > 10000)
          acceleration = -100;
        else if (abs(xEnc.read() - junc_pos) > 6500)
          acceleration = 0;
      }
      if (state == 5)
      {
        if (abs(xEnc.read() - junc_pos) < 6500)
        acceleration = 50;
      }
  }
  else if ((state == 1)|| (state == 6))
    {
      v1 = -2 * control - ang_control;
      v2 = (base - control + ang_control);
      v3 = (base + control - ang_control);
      k2.s(v2);
      k3.s(v3);
      k1.s(v1);
      if (state == 1)
      {
        if (((yEnc.read() > 10500) || (millis() - x > 2400)))
        {
          acceleration = -200;
          basedec = 1500;
          baseacc = 4000;
        }
      }
    }
    else if ((state == 3))
    {
      v1 = base + ang_control;
      v2 = (base / 2) - ang_control + control;
      v3 = (-base / 2) + ang_control + control;
      write_vel();
      if ((millis() - x) > 500)
      {
        base  = -400;
        v1 = base;
        v2 = base / 2;
        v3 = -base / 2;
        write_vel();
        while (abs(xEnc.read() - junc_pos) > 14050)
        {
        }
        state_change();
      }
    }
    else if (state == 4)
    {
      if ((digitalRead(8) == LOW) && (digitalRead(9) == LOW))
      {
        v1 = 0;
        v2 = base;
        v3 = base;
      }
      else if ((digitalRead(8) == HIGH) && (digitalRead(9) == LOW))
      {
        v1 = 0;
        v2 = +base / 5;
        v3 = -base / 5;
      }
      else if ((digitalRead(8) == LOW) && (digitalRead(9) == HIGH))
      {
        v1 = 0;
        v2 = -base / 5;
        v3 = +base / 5;
      }
      else if ((abs(xEnc.read() - x_finalpos) > 50) && (final_pos == true))
      {
        posx = xEnc.read();
        v1 = (x_finalpos - posx) / 8;
        v2 = (x_finalpos - posx) / 16;
        v3 = -(x_finalpos - posx) / 16;
        v1 = constrain(v1, -400, 400);
        v2 = constrain(v2, -400, 400);
        v3 = constrain(v3, -400, 400);
      }
      else
      {
        v1 = 0;
        v2 = 0;
        v3 = 0;
        if (final_pos == false)
        {
          final_pos = true;
          x_finalpos = xEnc.read();
        }
      }
      write_vel();
      if (millis() - x > 4000)
      {
        base = -500;
        junc_move_y();
        while ((analogRead(pin3) > 1000)||(analogRead(pin4) > 1000))
        {
        }
        if (dummy_state == true)
        {
          state = 6;
        }
        state_change();
      }
    }
    else if (state == 7)
    {
      digitalWrite(13,HIGH);
      v1 = base;
      v2 = -base;
      v3 = base;
      write_vel();
      while ((analogRead(pin3)>1000)&&(analogRead(pin4)>1000))
      {
      }
      if((analogRead(pin3)>1000))
      {
      v1 = base;
      v2 = -base;
      v3 = base;
      }
      k1.s(0);
      k2.s(0);
      k3.s(0);
      base = -200;
      v1 = base;
      v2 = base;
      v3 = base;
      write_vel();
      while ((analogRead(pin1)>1000)||(analogRead(pin2)>1000))
      {
      }
      state_change();
    }
  }
  else if (junc == true)
  {
    if (state == 0)
    {
      junc_move_x();
      while (analogRead(pin1) > 900)
      {}
      state_change();
    }
    else if ((state == 1))
    {
      base = base / 3;
      junc_move_y();
      while (analogRead(pin3) > 1000)
      {}
      delay(100);
      base = -400;
      junc_move_y();
      while ((analogRead(pin4) > 900) || (analogRead(pin4) < 300))
      {}
      state_change();
    }
   else if ((state == 5))
    {
      base = base / 4;
      junc_move_x();
      while (analogRead(pin2) > 800)
      {}
      v1 = 0; v2 = 0; v3 = 0;
      write_vel();
      base = 400;
      junc_move_x();
      while ((analogRead(pin2) > 1000) || (analogRead(pin1) > 1000))
      {}
      state_change();
    }
  }
}

void state_change()
{
  k1.s(0);
  k3.s(0);
  k2.s(0);
  if (state == 0)
  {
    baseacc = 4000;
    basedec = 0;
    base = 0;
    acceleration = 100;
    shaft = false;
    valread_y = 0;
    Kp1 = 0, Kd1 = 0;
    Kp2 = 0.3, Kd2 = 1.5;
    pval1 = analogRead(pin1);
    pval2 = target;
    val1 = pval1;
    val2 = pval2;
    x = millis();
  }
  else if ((state == 1))
  {
    baseacc = 3000;
    basedec = 0;
    base = 0;
    shaft = true;
    acceleration = 100;
    junc_pos = xEnc.read();
  }
  else if ((state == 2))
  {
    baseacc = 0;
    basedec = 0;
    base = 0;
    acceleration = 0;
    shaft = true;
    acceleration = 0;
    x = millis();
  }
  else if ((state == 3))
  {
    baseacc = 600;
    basedec = 0;
    base = 0;
    acceleration = 100;
    shaft = false;
    x = millis();
  }
  else if (state == 4)
  {
    shaft = true;
    acceleration = -120;
    basedec = -3500;
    baseacc = 0;
    base = 0;
  }
  else if ((state == 5))
  {
    baseacc = 2000;
    basedec = 0;
    base = 0;
    acceleration = 120;
    shaft = false;
  }
  if (state == 6)
  {
    acceleration = 200;
    baseacc = 1000;
    basedec = 0;
    base =0;
  }
  if (state==7)
  {
    acceleration = 0;
    basedec = 0;
    baseacc = 0;
  }
  if ((shaft == true)&&(state!= 0))
  {
    pval1 = analogRead(pin4);
    pval2 = analogRead(pin3);
  }
  else
  {
    pval1 = analogRead(pin4);
    pval2 = analogRead(pin3);
  }
  state++;
  junc = false;
}

void junction()
{
  if ((state != 2)&&(state != 5))
  {
    if (millis() - junc_t > 500)
    {
      if (shaft == true)
      {
        junc = true;
      }
      else if ((yEnc.read() > 20000) && ( state == 1))
      {
        junc = true;
      }
      else if (state == 6)
      {
        state = 1;
        junc = true;
        dummy_state = true;
      }
    }
    junc_t = millis();
  }
}

void junction_R()
{
  if (millis() - junc_t > 300)
  {
    if ((state == 5))
    {
      junc = true;
    }
  }
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

void Kangaroo_Init()
{
  Serial3.begin(19200);
  Serial2.begin(19200);
  k1.start();
  k1.home().wait();
  k2.start();
  k2.home().wait();
  k3.start();
  k3.home().wait();
}

int LSAread()
{

  posx = k1.getp().value();
  velx = k1.gets().value();
  posy = yEnc.read();
  vely = k2.getp().value();

  if (shaft == false)
  {
    if (analogRead(pin2) < 900)
    {
      valread_y = 1;
      acceleration = 150;
      Kp1 = -0.225, Kd1 = -0.65;
      Kp2 = 0.4, Kd2 = 0.3;
      pval2 = analogRead(pin2);
    }
    if (valread_y == 1)
    {
      val2 = analogRead(pin2);
    }
    else val2 = target;
    val1 = analogRead(pin1);
  }
  else
  {
    val1 = analogRead(pin4);
    val2 = analogRead(pin3);
  }

  if (val1 > 1000) val1 = pval1;
  if (val2 > 1000) val2 = pval2;
  val = (val1 + val2) / 2;
  pval1 = val1;
  pval2 = val2;

  if (val < 1000)
    return (target - val);
  else
    return (pdifference);

}
int angRead()
{
  ang = val1 - val2;
  return ang;
}

