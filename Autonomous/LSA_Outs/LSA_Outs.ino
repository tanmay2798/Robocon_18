#include<PWMFrequency.h>
int in0=A0;
int in1=A1;
int in2=A2;
int in3=A3;
int out0=11;
int out1=9;
int out2=5;
int out3=3;

void setup()
{
  pinMode(out0,OUTPUT);
  pinMode(out1,OUTPUT);
  pinMode(out2,OUTPUT);
  pinMode(out3,OUTPUT);
  SetPinFrequency(out0,1);
  SetPwmFrequency(out1,1);
  setPwmFrequency(out2,1);
  setPwmFrequency(out3,1);
}

void loop()
{
  analogWrite(out0,float(analogRead(in0))*0.165f);
  analogWrite(out1,float(analogRead(in1))*0.165f);
  analogWrite(out2,float(analogRead(in2))*0.165f);
  analogWrite(out3,float(analogRead(in3))*0.165f);
}
