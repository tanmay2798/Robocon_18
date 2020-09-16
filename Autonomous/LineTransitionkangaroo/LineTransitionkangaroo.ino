#include <Encoder.h>
#include <SoftwareSerial.h>
#include <Kangaroo.h>
#include <Sabertooth.h>
boolean junction=false,shaft=false;
float target=475,control=0,v1=0,v2=0,v3=0,t=0,tc=0,ta=2,base=0,x=0;
long int encpos;
float Kp=-0.45,Kd=-1.3;
float difference=0,pdifference=0,differential=0;
int pin1=A0,pin2=A1;
long basemax=0,acceleration=75,deceleration=150;
KangarooSerial K1(Serial1);
KangarooSerial K2(Serial2);
KangarooChannel k1(K1, '1');
KangarooChannel k2(K2, '2');
KangarooChannel k3(K1, '2');
Encoder freeW(2, 3);
void Junction()
{
  junction=true;
}
int LSAread(int pin)
{
  Serial.println(analogRead(pin));
  if(analogRead(pin)<1000)
    return(target-analogRead(pin));
  else
    return(pdifference);
}
void setup()
{
  Serial.begin(38400);
  Serial1.begin(19200);
  Serial2.begin(19200);
  k1.start();
  k1.home().wait();
  k2.start();
  k2.home().wait();
  k3.start();
  k3.home().wait();
  basemax = 1500;
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(21),Junction,RISING);
  junction=false;
}

void loop()
{
  
  if((millis()-tc)>=ta)
  {
    tc=millis();
    base=constrain(base+acceleration,0,basemax);
  }
  if(junction==false)
  {
    if(shaft==false)
      difference=LSAread(pin1);
    else
      difference=LSAread(pin2);
  }
  if(millis()!=t)
  {
    differential=(difference-pdifference)/(millis()-t);
    t=millis();
  }
  control=Kp*difference+Kd*differential;
  pdifference=difference;
  if(junction==false)
  {
    if(shaft==false)
    {
     
      v1 = -2*control;
      v2 = (base+control);
      v3 = (base-control);  
      
      
      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
      
    }
    
    else
    {
      
      v1 = (base+control);
      v2 = (base-control)/2;
      v3 = -(base-control)/2;
       
      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
     
    }
  }
  else
  {
    while(analogRead(A1)>500)
    {
      base=constrain(base-deceleration,500,basemax);
      k2.s(base);
      k3.s(base);
    }
    k1.s(0);
    k2.s(0);
    junction=false;
    shaft=true;
    Kp=1;
    Kd=1;
    basemax=1500;
    delay(500);
  }
}
