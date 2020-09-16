#include <Encoder.h>
#include <SoftwareSerial.h>
#include <Kangaroo.h>
#include <Sabertooth.h>
boolean junction=false,shaft=false;
float target=475,ang_control=0,control=0,v1=0,v2=0,v3=0,t=0,tc=0,ta=2,base=0,cnt=0, flag = 0, ltime = 2000;
long int encpos;
int val=0,val1=0,val2=0,pval1=0, pval2 = 0, ang =0;
float Kp1=-0.225,Kd1=0.65;
float Kp2=-0.4,Kd2=-0.3;
float difference=0,pdifference=0,differential=0;
float ang_difference=0,ang_pdifference=0,ang_differential=0;
int pin1=A0,pin2=A1;
long basedec=0,baseacc=0,acceleration=50,deceleration=150,x =0;
KangarooSerial K1(Serial1);
KangarooSerial K2(Serial2);
KangarooChannel k1(K1, '1');
KangarooChannel k2(K2, '2');
KangarooChannel k3(K1, '2');
Encoder freeW(2, 3);

int LSAread()
{
  val1 = analogRead(pin1);
  val2 = analogRead(pin2);
  if (val1 > 1000) val1 = pval1; 
  if (val2 > 1000) val2 = pval2;
  val = (val1+val2)/2;   
  pval1 = val1;
  pval2 = val2; 
  
  if(val<1000)
    return(target-val);
  else
    return(pdifference);
      
}
int angRead()
{
 ang = val1 - val2;
 return ang;
}
void setup()
{
  Serial1.begin(19200);
  Serial2.begin(19200);
  k1.start();
  k1.home().wait();
  k2.start();
  k2.home().wait();
  k3.start();
  k3.home().wait();
  basedec = 0;
  baseacc = 3500;
  pinMode(21, INPUT_PULLUP);
  junction=false;
  pval1 = analogRead(pin1);
  pval2 = analogRead(pin2);
  delay(100);
  x = millis();
  
}

void loop()
{
    base=constrain(base+acceleration,basedec,baseacc);
    if (flag ==0)
  acceleration=constrain(acceleration+5,50,90);
      difference=LSAread();
      ang_difference = angRead(); 

  if(millis()!=t)
  {
    differential=(difference-pdifference)/(millis()-t);
    ang_differential=(ang_difference-ang_pdifference)/(millis()-t);
    t=millis();
  }
  control=Kp1*difference+Kd1*differential;
  ang_control = Kp2*ang_difference+Kd2*ang_differential;
  pdifference=difference;
  ang_pdifference=ang_difference;
  
      v1 = -2*control+ang_control;
      v2 = (base+control-ang_control);
      v3 = (base-control+ang_control);  
     
      k3.s(v3);
      k2.s(v2);
      k1.s(v1);
      if ((millis() - x)>ltime)
      {
          acceleration = -acceleration;
          ltime = 4000;
          if (flag == 0)
          {
            basedec = -3500;
            flag = 1;
          }
          x = millis();
      }
}
