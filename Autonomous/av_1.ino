#include <Kangaroo.h>
#include <Sabertooth.h>
#include <Encoder.h>

float target = 465, ang_control = 0, control = 0, v1 = 0, v2 = 0, v3 = 0;
float t = 0, x = 0, cnt = 0, flag = 0, state = 0, valread_y = 0, junc_R = 0, detect = 2, junc_t = 0, neg_vel = 0;
long int posx = 0, velx  = 0, posy = 0, vely  = 0, prev = 0, junc_pos = 0, x_finalpos = 0;
int val = 0, val1 = 0, val2 = 0, pval1 = 0, pval2 = 0, ang = 0, i = 0;
bool junc = false, shaft = true, final_pos = false, dummy_state = false, manual = true;
float Kp1 = -0.22, Kd1 = -0.17;
float Kp2 = 0.14, Kd2 = 0.09;
float difference = 0, pdifference = 0, differential = 0;
float ang_difference = 0, ang_pdifference = 0, ang_differential = 0;
int pin1 = A0, pin2 = A1, pin3 = A2, pin4 = A3;
long basedec = 0, baseacc = 0, acceleration = 0, deceleration = 0, base = 0;
long duration, inches, cm;
KangarooSerial K1(Serial3);
KangarooSerial K2(Serial2);
KangarooChannel k3(K1, '1');
KangarooChannel k1(K2, '2');
KangarooChannel k2(K1, '2');
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
  pinMode(11, OUTPUT);
  pinMode(12, INPUT);
  pinMode(13, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(20), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(21), junction_R, RISING);
  Kangaroo_Init();
  pval1 = analogRead(pin4);
  pval2 = analogRead(pin3);

  basedec = 0;
  baseacc = 1450;
  acceleration = 30;
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

  if (shaft ==  true)
  {
    Kp1 = -0.22, Kd1 = -0.17;
    Kp2 = 0.14, Kd2 = 0.09;
  }
  else if ((shaft == false) && (state != 1))
  {
    Kp1 = -0.085, Kd1 = -0.193;
    Kp2 = 0.10, Kd2 = 0.08;
  }

  control = Kp1 * difference + Kd1 * differential;
  ang_control = Kp2 * ang_difference + Kd2 * ang_differential;
  pdifference = difference;
  ang_pdifference = ang_difference;

  if (junc == false)
  {
    if ((state == 0) || (state == 2) || (state == 5) || (state == 9))
    {
      v1 = base + ang_control;
      v2 = (base / 2) - ang_control + control;
      v3 = (-base / 2) + ang_control + control;

      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
      if (state == 2)
      {
        if ((abs(xEnc.read() - junc_pos)) > 6000)
        {
          v1 = 0;
          v2 = 0;
          v3 = 0;
          write_vel();
          state_change();
        }
        else if (abs(xEnc.read() - junc_pos) > 4200)
          acceleration = -100;
        else if (abs(xEnc.read() - junc_pos) > 3000)
          acceleration = 0;
      }
      if (state == 5)
      {
        if (abs(xEnc.read() - junc_pos) < 6500)
          acceleration = 21;
      }
    }
    else if ((state == 1) || (state == 6))
    {
      v1 = -2 * control - ang_control;
      v2 = (base - control + ang_control);
      v3 = (base + control - ang_control);
      k2.s(v2);
      k3.s(v3);
      k1.s(v1);
      if (state == 1)
      {
        if (((yEnc.read() > 10000) || (millis() - x > 2400)))
        {
          acceleration = -75;
          basedec = 625;
          baseacc = 1650;
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
        base  = -300;
        v1 = base;
        v2 = base / 2;
        v3 = -base / 2;
        write_vel();
        while (abs(xEnc.read() - junc_pos) > 6100)
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
      else
      {
        if (final_pos == false)
        {
          final_pos = true;
          x_finalpos = xEnc.read();
          delay(100);
        }
        else if ((abs(xEnc.read() - x_finalpos) > 100) && (final_pos == true))
        {
          posx = xEnc.read();
          v1 = (x_finalpos - posx) / 2;
          v1 = constrain(v1, -200, 200);
          v2 = v1 / 2;
          v3 = -v2;
        }
        else
        {
          k1.s(0);
          k2.s(0);
          k3.s(0);
          if ( flag == 0)
          {
            flag = 1;
            x = millis();
            manual = false;
          }
        }
      }
//      else if ((abs(xEnc.read() - x_finalpos) > 100) && (final_pos == true))
//      {
//        posx = xEnc.read();
//        v1 = (x_finalpos - posx) / 2;
//        v1 = constrain(v1, -200, 200);
//        v2 = v1 / 2;
//        v3 = -v2;
//      }
//      else
//      {
//        v1 = 0;
//        v2 = 0;
//        v3 = 0;
//        if (final_pos == false)
//        {
//          final_pos = true;
//          x_finalpos = xEnc.read();
//          delay(100);
//        }
//        manual = false;
//        x = millis();
//      }
//      write_vel();

      //      if ((digitalRead(8) == HIGH) && (digitalRead(9) == HIGH)&&(abs(xEnc.read() - x_finalpos) < 50))
      //      {
      //        for (cnt = 0; cnt < 5; cnt++)
      //        {
      //          pinMode(10, OUTPUT);
      //          digitalWrite(10, LOW);
      //          delayMicroseconds(2);
      //          digitalWrite(10, HIGH);
      //          delay(10);
      //          digitalWrite(10, LOW);
      //
      //          pinMode(10, INPUT);
      //          duration = pulseIn(10, HIGH);
      //          cm = microsecondsToCentimeters(duration);
      //          if ((cm > 20) && (cm < 200))
      //          {
      //            manual = true;
      //            break;
      //          }
      //          if (cnt == 4)
      //            manual = false;
      //        }
      //        if (manual == true)
      //        {
      //          digitalWrite(11, HIGH);
      //          delay(100);
      //          digitalWrite(11, LOW);
      //          while (digitalRead(12) == LOW)
      //          {
      //            k1.s(0);
      //            k2.s(0);
      //            k3.s(0);
      //          }
      //        }
      if ((manual == false) && (millis() - x > 1000))
      {
        base = -210;
        junc_move_y();
        while ((analogRead(pin3) > 900) || (analogRead(pin4) > 900))
        {
        }
        k1.s(0);
        k2.s(0);
        k3.s(0);
        if (dummy_state == true)
        {
          base = -340;
          junc_move_x();
          x = millis();
          while (millis() - x < 1500)
          {
          }
          state = 6;
        }
        state_change();
      }
    }
    else if (state == 7)
    {
      base = -210;
      acceleration = 0;
      v1 = base;
      v2 = -base;
      v3 = base;
      write_vel();
      delay(1000);
      while ((analogRead(pin3) > 980))
      {
      }
      while ((analogRead(pin3) > 980) || (analogRead(pin4) > 980))
      {
        k1.s(0);
        k2.s(0);
        k3.s(0);
        v1 = -225;
        v2 = 81;
        v3 = -291;
        write_vel();
        while ((analogRead(pin4) > 980))
        {
        }
        k1.s(0);
        k2.s(0);
        k3.s(0);
        v1 = 225;
        v3 = 81;
        v2 = -291;
        write_vel();
        while ((analogRead(pin3) > 980))
        {}
      }
      k1.s(0);
      k2.s(0);
      k3.s(0);
      state_change();
    }
    else if (state == 8)
    {
      //      digitalWrite(13, HIGH);
      //      manual = true;
      //      while (manual == false)
      //      {
      //        for(cnt = 0; cnt<5; cnt++)
      //        {
      //        pinMode(10, OUTPUT);
      //        digitalWrite(10, LOW);
      //        delayMicroseconds(2);
      //        digitalWrite(10, HIGH);
      //        delay(10);
      //        digitalWrite(10, LOW);
      //
      //        pinMode(10, INPUT);
      //        duration = pulseIn(10, HIGH);
      //        cm = microsecondsToCentimeters(duration);
      //        if ((cm > 20) && (cm < 200))
      //        {
      //          manual = true;
      //          break;
      //        }
      //        if (cnt == 4)
      //        manual = false;
      //        }
      //      }
      //      digitalWrite(13, LOW);
      //      delay(100);
      //      manual == false;
      //      while (manual == true)
      //      {
      //        for(cnt = 0;cnt<5;cnt++)
      //        {
      //        pinMode(10, OUTPUT);
      //        digitalWrite(10, LOW);
      //        delayMicroseconds(2);
      //        digitalWrite(10, HIGH);
      //        delay(10);
      //        digitalWrite(10, LOW);
      //
      //        pinMode(10, INPUT);
      //        duration = pulseIn(10, HIGH);
      //        cm = microsecondsToCentimeters(duration);
      //        if ((cm < 20) || (cm > 200))
      //        {
      //          manual = false;
      //        }
      //        if(cnt ==4)
      //        {
      //          manual = true;
      //        }
      //        }
      //      }
      //      delay(100);
      digitalWrite(13, HIGH);
      state_change();
    }
    else if ((state == 10))
    {
      v1 = base + ang_control;
      v2 = (base / 2) - ang_control + control;
      v3 = (-base / 2) + ang_control + control;
      write_vel();
    }
  }
  else if (junc == true)
  {
    if (state == 0)
    {
      junc_move_x();
      while (analogRead(pin1) > 900)
      {}
      k1.s(0);
      k2.s(0);
      k3.s(0);
      base = -base/3;
      junc_move_x();
      delay(100);
      while (analogRead(pin1) > 900)
        digitalWrite(13, LOW);
      state_change();
    }
    else if ((state == 1))
    {
      base = base/3;
      junc_move_y();
      while (analogRead(pin3) > 980)
      {}
      delay(100);
      base = -300;
      junc_move_y();
      while ((analogRead(pin4) > 900) || (analogRead(pin4) < 200)||(analogRead(pin3) > 900) || (analogRead(pin3) < 200))
      {}
      state_change();
    }
    else if ((state == 5))
    {
      base = base / 4;
      junc_move_x();
      while ((analogRead(pin2) > 900) || (analogRead(pin1) > 900))
      {}
      k1.s(0);
      k2.s(0);
      k3.s(0);
      delay(100);
      base = 170;
      junc_move_x();
      while ((analogRead(pin1) > 980) || (analogRead(pin2) > 980))
      {}
      k1.s(0);
      k2.s(0);
      k3.s(0);
      state_change();
    }
    else if (state == 9)
    {
      if (junc_R < 5)
      {
        junc_R++;
        junc = false;
        if (junc_R > 4)
        {
          baseacc = -625;
          acceleration = 62;
        }
      }
      else if (junc_R == 5)
      {
        base = base / 3;
        junc_move_x();
        while (analogRead(pin1) > 980)
        {
        }
        v1 = 0;
        v2 = 0;
        v3 = 0;
        write_vel();
        state_change();
      }
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
    baseacc = 1650;
    basedec = 0;
    base = 0;
    acceleration = 42;
    shaft = false;
    valread_y = 0;
    Kp1 = 0, Kd1 = 0;
    Kp2 = 0.10, Kd2 = 0.1;
    pval1 = analogRead(pin1);
    pval2 = target;
    val1 = pval1;
    val2 = pval2;
    x = millis();
  }
  else if ((state == 1))
  {
    baseacc = 1250;
    basedec = 0;
    base = 0;
    shaft = true;
    acceleration = 42;
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
    baseacc = 350;
    basedec = 0;
    base = 0;
    acceleration = 42;
    shaft = false;
    x = millis();
    final_pos = false;
    manual = true;
  }
  else if (state == 4)
  {
    shaft = true;
    acceleration = -50;
    basedec = -1450;
    baseacc = 0;
    base = 0;
  }
  else if ((state == 5))
  {
    baseacc = 840;
    basedec = 0;
    base = 0;
    acceleration = 50;
    x = millis();
    shaft = false;
  }
  if (state == 6)
  {
    acceleration = 84;
    baseacc = 420;
    basedec = 0;
    base = 0;
  }
  if (state == 7)
  {
    acceleration = 0;
    base = 0;
    basedec = 0;
    baseacc = 0;
    manual = false;
  }
  if (state == 8)
  {
    acceleration = -30;
    base = 0;
    basedec = -840;
    baseacc = 0;
    shaft = true;
    junc_R = 0;
  }
  if (state == 9)
  {
    acceleration = 0;
    base = 0;
    basedec = 0;
    baseacc = 0;
    shaft = true;
  }
  if ((shaft == false) && (state != 0))
  {
    pval1 = analogRead(pin1);
    pval2 = analogRead(pin2);
  }
  else if (state != 0)
  {
    pval1 = analogRead(pin4);
    pval2 = analogRead(pin3);
  }
  else if (state == 0)
  {
    pval1 = analogRead(pin1);
    pval2 = target;
  }
  state++;
  junc = false;
}

void junction()
{
  digitalWrite(13, HIGH);
  if ((state != 2) && (state != 5) && (state != 9))
  {
    if (millis() - junc_t > 500)
    {
      if (shaft == true)
      {
        junc = true;
      }
      else if ((millis() - x > 3200) && ( state == 1))
      {
        junc = true;
      }
      else if (((millis() - x) > 500) && (state == 6))
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
  if (millis() - junc_t > 150)
  {
    if ((state == 5) || (state == 9))
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
      acceleration = 62;
      Kp1 = -0.085, Kd1 = -0.193;
      Kp2 = 0.10, Kd2 = 0.08;
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

  if (val1 > 980) val1 = pval1;
  if (val2 > 980) val2 = pval2;
  val = (val1 + val2) / 2;
  pval1 = val1;

  pval2 = val2;

  if (val < 980)
    return (target - val);
  else
    return (pdifference);

}
int angRead()
{
  ang = val1 - val2;
  return ang;
}
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}
