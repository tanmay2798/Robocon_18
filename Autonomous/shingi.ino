#include <Kangaroo.h>
#include <Sabertooth.h>
#include <Encoder.h>

float target = 475, ang_control = 0, control = 0, v1 = 0, v2 = 0, v3 = 0;
float t = 0, x = 0, cnt = 0, flag = 0, state = 0, valread_y = 0, detect = 2, j_left = 0;
long int posx = 0, velx  = 0, posy = 0, vely  = 0, prev = 0;
int val = 0, val1 = 0, val2 = 0, pval1 = 0, pval2 = 0, ang = 0, i = 0;
bool junc = false, shaft = true;
float Kp1 = -0.225, Kd1 = -0.65;      //float Kp1=-0.225,Kd1=-0.65;
float Kp2 = 0.4, Kd2 = 0.3;           //float Kp2=0.4,Kd2=0.3;
float difference = 0, pdifference = 0, differential = 0;
float ang_difference = 0, ang_pdifference = 0, ang_differential = 0;
int pin1 = A0, pin2 = A1, pin3 = A2, pin4 = A3;
long basedec = 0, baseacc = 0, acceleration = 0, deceleration = 0, base = 0;

KangarooSerial K1(Serial1);
KangarooSerial K2(Serial2);
KangarooChannel k1(K1, '1');
KangarooChannel k2(K2, '2');
KangarooChannel k3(K1, '2');
Encoder yEnc(2, 3);

void setup()
{
  Serial.begin(9600);

  Kangaroo_Init();

  pinMode(13, OUTPUT);

  pval1 = analogRead(pin4);
  pval2 = analogRead(pin3);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(21), junction, RISING);

  basedec = 0;
  baseacc = 3500;
  acceleration = 70;
  junc = false;    // state 0
  shaft = true;    // state 0

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
    if (shaft == true)
    {
      v1 = base + ang_control;
      v2 = base / 2 - ang_control + control;
      v3 = -base / 2 + ang_control + control;

      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
    }
    if (shaft == false)
    {
      v1 = -2 * control + ang_control;
      v2 = (base + control + ang_control);
      v3 = (base - control - ang_control);
      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
      if ((yEnc.read() > 12000))
      {
        acceleration = -200;
        basedec = 1200;
        baseacc = 4000;
      }
    }
  }
  else
  {
    if (shaft == true)
    {
      while (analogRead(pin1) > 1000)
      {
        v1 = base;
        v2 = base / 2;
        v3 = -(base) / 2;
        k3.s(v3);
        k2.s(v2);
        k1.s(v1);
      }
      k3.s(0);
      k2.s(0);
      k1.s(0);
      baseacc = 4000;
      basedec = 0;
      acceleration = 120;
      shaft = false;
      junc = false;
      valread_y = 0;
      Kp1 = 0, Kd1 = 0;
      Kp2 = 0.5, Kd2 = 1.3;
      delay(500);
    }
    else if (shaft == false)
    {
      while ((analogRead(pin4) > 1000)&&(analogRead(pin3) > 1000))
      {
        v1 = 0;
        v2 = base;
        v3 = base;
        k3.s(v3);
        k2.s(v2);
        k1.s(v1);
      }
      k3.s(0);
      k2.s(0);
      k1.s(0);
      while ((analogRead(pin4) > 800)&&(analogRead(pin3) > 800))
      {
        v1 = 0;
        v2 = -base/4;
        v3 = -base/4;
        k3.s(v3);
        k2.s(v2);
        k1.s(v1);
      }
      baseacc = 4000;
      basedec = 0;
      acceleration = 120;
      shaft = true;
      junc = false;
      delay(500);
    }
  }
}

void junction()
{
  if (shaft == true)
  junc = true;
  else if (yEnc.read()>22000)
  junc = true;
}

void Kangaroo_Init()
{
  Serial1.begin(19200);
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
    if (analogRead(pin2) < 1000)
    {
      valread_y = 1;
      Kp1 = -0.225, Kd1 = -0.65;
      Kp2 = 0.4, Kd2 = 0.3;
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
