#include <Kangaroo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L0X2.h>
#define dis 600

VL53L0X  sensor1;
VL53L0X2 sensor2;

float angcontrol=0,distcontrol=0,v1=0,v2=0,v3=0,t=0,tc=0,base=0;
float Kp1=3.0,Kd1=0,Kp2=3.0,Kd2=0;
float angdif=0,p_angdif=0,angdifferential=0;
float distdif=0,p_distdif=0,distdifferential=0;
long basemax = 0;
long l1=0, l2=0, dist = 250, ang = 0;

KangarooSerial K1(Serial2);
KangarooSerial K2(Serial1);
KangarooChannel k1(K1, '1');
KangarooChannel k2(K2, '2');
KangarooChannel k3(K1, '2');

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

  Wire.begin();

  sensor1.init();
  sensor1.setTimeout(500);
  sensor2.init();
  sensor2.setTimeout(500);

  sensor1.startContinuous();
  sensor2.startContinuous();
  
  basemax = 4500;

}
void loop()
{
  if((millis()-tc)>=2 && base<=basemax)
  {
    tc=millis();
    base=base+50;
  }

  l1 = Serial.print(sensor1.readRangeContinuousMillimeters());
  l2 = Serial.print(sensor2.readRangeContinuousMillimeters());
  
  angdif = l1-l2-ang; 
  distdif = ((l1+l2)/2)-dist;  
  
  if(millis()!=t)
  {
    angdifferential=(angdif - p_angdif)/(millis()-t);
    distdifferential=(distdif - p_distdif)/(millis()-t);
    t=millis();
  }
  
  angcontrol=Kp1*angdif+Kd1*angdifferential;
  distcontrol=Kp2*distdif+Kd2*distdifferential;
  p_angdif=angdif;
  p_distdif=distdif;
  
  v1=(-base/2)+angcontrol+distcontrol;
  v2=(base+angcontrol);
  v3=(base/2)+angcontrol-distcontrol;
  
  k1.s(v1);
  k2.s(v2);
  k3.s(v3);
  
  Serial.print("angle =");
  Serial.print(angdif);
  Serial.print("distance =");
  Serial.print(distdif+250);
  Serial.print("v1=");
  Serial.print(v1);
  Serial.print("v2=");
  Serial.println(v2);
  Serial.print("v3=");
  Serial.println(v3);
}
