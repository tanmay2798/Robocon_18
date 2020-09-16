// Speed Control Sample for Kangaroo
// Copyright (c) 2013 Dimension Engineering LLC
// See license.txt for license details.
#include <Kangaroo.h>

// Arduino TX (pin 11) goes to Kangaroo S1
// Arduino RX (pin 10) goes to Kangaroo S2
// Arduino GND         goes to Kangaroo 0V
// Arduino 5V          goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Arduino)

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
KangarooSerial  K1(Serial1);
KangarooSerial  K2(Serial2);
KangarooChannel M1(K1, '2');
KangarooChannel M2(K2, '1');
KangarooChannel M3(K2, '2');

int vel1 = 0, vel2 = 0, vel3 = 0;

void setup()
{
  Serial2.begin(19200);
  Serial1.begin(19200);
  
  M1.start();
  M1.home().wait();
  M2.start();
  M2.home().wait();
  M3.start();
  M3.home().wait();
 
delay(1000);
      
 
}

// .wait() waits until the command is 'finished'. For speed, this means it reached near the
// requested speed. You can also call K1.s(speed); without .wait() if you want to command it
// but not wait to get up to speed. If you do this, you may want to use K1.getS().value()
// to check progress.
void loop()
{
 
 ramp(0,1250,-1250,50);

 delay(4000);

 ramp(0,0,0,5);

 delay(3000);
  
}

void ramp(int v1, int v2, int v3, int t)
{
 int diff1 = (v1 - vel1)/t;
 int diff2 = (v2 - vel2)/t;
 int diff3 = (v3 - vel3)/t;

 for (int i=1; i<=t; i++)
 {
  M2.s(vel2 + (i*diff2));
  M3.s(vel3 + (i*diff3)); 
  M1.s(vel1 + (i*diff1));
  delay(10);
 }

 vel1 = v1;
 vel2 = v2;
 vel3 = v3;
}

