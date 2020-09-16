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

void junc_move_x();
void junc_move_y();
void state_change();
void junction();

void setup()
{
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  attachInterrupt(digitalPinToInterrupt(20), junction, RISING);
  attachInterrupt(digitalPinToInterrupt(21), junction, RISING);

  Serial.begin(9600);

  Kangaroo_Init();

  pinMode(13, OUTPUT);

  pval1 = analogRead(pin4);
  pval2 = analogRead(pin3);

  basedec = 0;
  baseacc = 3500;
  acceleration = 40;
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
    if ((state == 0) || (state == 2) || (state == 4) || (state == 6))
    {
      v1 = base + ang_control;
      v2 = (base / 2) - ang_control + control;
      v3 = (-base / 2) + ang_control + control;

      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
    }
    else if ((state == 1) || (state == 5))
    {
      v1 = -2 * control - ang_control;
      v2 = (base - control + ang_control);
      v3 = (base + control - ang_control);
      k2.s(v2);
      k3.s(v3);
      k1.s(v1);
      if ((yEnc.read() > 9500) && (state == 1))
      {
        acceleration = -100;
        basedec = 1000;
        baseacc = 4000;
      }
    }
    else if (((state == 3) || (state == 7)) && ((millis() - x) > 4000))
    {
      v1 = -2 * control - ang_control;
      v2 = (base - control + ang_control);
      v3 = (base + control - ang_control);
      state_change();
    }
  }
  else if (junc == true)
  {
    
    if (state == 0)
    {
      while (analogRead(pin1) > 900)
      {
        junc_move_x();
      }
      state_change();
    }
    else if ((state == 1) || (state == 5))
    {
      while ((analogRead(pin4) > 1000))
      {
        junc_move_y();
      }
      k3.s(0);
      k2.s(0);
      k1.s(0);
      base = -200;
       while ((analogRead(pin4) > 1000))
      {
        junc_move_y();
      }
      state_change(); 
    }

    else if ((state == 2) || (state == 4) || (state == 6))
    {
      while ((analogRead(pin1) > 900) || (analogRead(pin2) > 900))
      {
        junc_move_x();
      }
      state_change();
    }
  }
}

