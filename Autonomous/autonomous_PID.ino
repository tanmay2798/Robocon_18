#include <Kangaroo.h>
#include <Sabertooth.h>
float target=465,ang_control=0,control=0,v1=0,v2=0,v3=0,t=0,tc=0,ta=2,base=0,cnt=0, flag = 0, state = 0;
long int pos = 0,vel  = 0;
int val=0,val1=0,val2=0,pval1=0, pval2 = 0, ang =0;
//float Kp1 = -0.18, Kd1 = -0.1;
//float Kp2 = 0.13, Kd2 = 0.1; 
float Kp1 = -0.11, Kd1 = -0.29;      //float Kp1=-0.225,Kd1=-0.65;
float Kp2 = 0.129, Kd2 = 0.1;
float difference=0,pdifference=0,differential=0;
float ang_difference=0,ang_pdifference=0,ang_differential=0;
int pin1=A0,pin2=A1;
long basedec=-1650,baseacc=1600,acceleration=40,deceleration=0,x =0;
KangarooSerial K1(Serial3);
KangarooSerial K2(Serial2);
KangarooChannel k3(K1, '1');
KangarooChannel k1(K2, '2');
KangarooChannel k2(K1, '2');

int LSAread()
{
  
  pos = k3.getp().value();
  vel = k3.gets().value();
  val1 = analogRead(pin1);
  val2 = analogRead(pin2);
  if (val1 > 980) val1 = pval1; 
  if (val2 > 980) val2 = pval2;
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
  Serial.begin(38400);
  Serial3.begin(19200);
  Serial2.begin(19200);
  k1.start();
  k1.home().wait();
  k2.start();
  k2.home().wait();
  k3.start();
  k3.home().wait();
  pval1 = analogRead(pin1);
  pval2 = analogRead(pin2);
  delay(100);
  x = millis();
    
}

void loop()
{
  x = millis();
    base=constrain(base+acceleration,basedec,baseacc);
//    if (flag == 0)
//    acceleration=constrain(acceleration+5,50,90);
      difference=LSAread();
      ang_difference = angRead(); 

  if(millis()!=t)
  {
    differential=(pdifference-difference)/(millis()-t);
    ang_differential=(ang_difference-ang_pdifference)/(millis()-t);
    t=millis();
  }
  control=Kp1*difference+Kd1*differential;
  ang_control = Kp2*ang_difference+Kd2*ang_differential;
  pdifference=difference;
  ang_pdifference=ang_difference;
  
      v1 = -2 * control - ang_control;
      v2 = (base - control + ang_control);
      v3 = (base + control - ang_control);
      k2.s(v2);
      k3.s(v3);
      k1.s(v1);
     
//      if ((pos > 1300)&&(state==0))
//      {
//          state++;
//          acceleration = -90;
//          flag = 1;  
//      }
//      else if ((vel<0)&&(state==1))
//      {
//          state++; 
//          acceleration = 0; 
//          x = millis();
//          basedec = 0;
//          baseacc = 0;
//      }
//      else if ((millis()-x)>4000&&(state==2))
//      {
//          control = 0;
//          ang_control = 0;
//          state++; 
//          acceleration = -90; 
//          basedec = -3500;
//          baseacc = 3500;
//      }
//      else if ((pos<0)&&(state==3))
//      {
//          state++; 
//          acceleration = 90; 
//      }
//      else if ((vel>0)&&(state==4))
//      {
//          state++; 
//          acceleration = 0;
//          x = millis();
//          basedec = 0;
//          baseacc = 0;
//      }
//      else if ((millis()-x)>4000&&(state==5))
//      {
//          control = 0;
//          ang_control = 0;
//          state = 0;
//          acceleration = 90;
//          basedec = -3500;
//          baseacc = 3500;
//      }
//            
}
