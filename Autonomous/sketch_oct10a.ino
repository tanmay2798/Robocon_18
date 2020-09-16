#include <Sabertooth.h>



#include <SoftwareSerial.h>
//#include <SoftwareSerial>
#include <SabertoothSimplified.h>
//#include <Kangaroo.h>
//char buffer1[32];
//char buffer2[32];
//SoftwareSerial Serial1(NO; // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST1(Serial1); // Use SWSerial as the serial port.
//SoftwareSerial Serial2(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST2(Serial2); // Use SWSerial as the serial port.
/*KangarooSerial  K(Serial2);
KangarooChannel k1m1(K, '1');
KangarooChannel k1m2(K, '2');
//SoftwareSerial  Serial2
KangarooSerial  Ka(Serial1);
KangarooChannel k2m1(Ka, '1');*/
float kpw=1.0,kdw=1.0,kpx=1.0,kdx=1.0;
float differential,difference;
float wmax=100.0,velmaxy=25.0,theta=0,base=1.0,r=30,velmaxx=25.0,lastdiff=0,vx=0,vy=10,w=0,d0=0,d00=0,t=1,d1,d2,t1=0.0,t2=0.0;
float dis1[20];//practise se nikaalni hai
float dis2[20];//practise se nikaalni hai

void setup() {
  Serial1.begin(19200);
  Serial2.begin(19200);
  Serial.begin(9600);
}
void loop() {
  
  //delay(10);
  int i=0;
  for(i=0;i<10;i++){
  float d1=(analogRead(A0)/1023.0)*40;
  dis1[i]=d1;
  
  //delay(10);
  float d2=(analogRead(A3)/1023.0)*40;
  dis2[i]=d2;
  }
  d1=dis1[10];
  d2=dis2[10];
  t1=millis();
  t=t1-t2;
  if (d1!=d2){
    
    difference=(d1-d2)/2;
    differential=(difference-(d0*0.5-d00*0.5))/t;
  if (abs((d1-d2)/(2*base*t))<=wmax){
     w=-1.0*(d1-d2)*kpw/(2*base*t)+kdw*differential*t/base;}
  else if(abs(difference/(base*t))<wmax){
     w=-wmax*abs(difference/(base*t))/((d1-d2)/(2*base*t));}
           }
 t1=millis();
  t=t1-t2;
 if(d1==d2 && d1-d0!=0){
  float difference=d1-d0;
  float differential=(difference-lastdiff)/t;
  if (abs(difference/t)<=velmaxx){
  vx=(-1*difference/t)*kpx+differential*kdx*t;

   }
   else if(abs(difference/t)>velmaxx){
    vx=-1*abs(difference/t)*velmaxx/(difference/t);}}
 vy=velmaxy;
float  v1=-vx*0.577+vy*0.333+w*r*(-0.333); 
float  v2=vx*0.577+0.333*vy+(-1)*w*r*0.333;
 float v3=0.666*vy+0.333*w*r;

Serial.print(d1);
Serial.print("    ");
Serial.print(d2);
Serial.print("    ");
Serial.print(vx);
Serial.print("    ");
Serial.print(w);
Serial.print("    ");

Serial.print(v1);
Serial.print("    ");
Serial.print(v2);
Serial.print("    ");
Serial.print(v3);
Serial.print("    ");
Serial.println("\t");
//k1m1.s(v1);
//k1m2.s(v3);
//k2m1.s(v2);
ST1.motor(1,v2);
ST1.motor(2,v1);
ST2.motor(2,v3);

  /*-0.5771672630728385  0.3334487667859479  -0.333448766785948
2 0.5771672630728385  0.3332178998807187  -0.33321789988071876
3 0 0.6666666666666666  0.3333333333333333
   * 
   */
  
 d0=d1;
 d00=d2;
 lastdiff=difference;
 t2=millis();

}
