#include <Sabertooth.h>
Sabertooth golden_rack(128, Serial3);

int throw_cnt_tz3 = 0;
int golden_rack_w = 40;
int limit_golden = 22;

void setup() {
Serial3.begin(9600);
}

void loop() {
 int gold_w = golden_rack_w;

  golden_rack.motor(2,gold_w);
  delay(200);
  for(int i = 0; i < throw_cnt_tz3+2; i++)
  {
    golden_rack.motor(2,gold_w);
    delay(100);
    while (digitalRead(limit_golden) == LOW)
    {
      delay(1);
    }
  }

  delay(200);
  golden_rack.motor(2,0);
  gold_w = -gold_w;
  delay(200);
  for(int i = 0; i < throw_cnt_tz3+2; i++)
  {
    golden_rack.motor(2,gold_w);
    delay(200);
    while (digitalRead(limit_golden) == LOW)
    {
      delay(1);
    }
  }

  delay(200);
  golden_rack.motor(2,0);
  gold_w = -gold_w/2;
  golden_rack.motor(2,gold_w);
  
  while(true){
    while (digitalRead(limit_golden) == LOW){
    delay(1);
    }
    golden_rack.motor(2,0);
    delay(100);
    if (digitalRead(limit_golden) == HIGH){
      break;
    }else{
      gold_w = -gold_w/2;
      golden_rack.motor(2,gold_w);
    }
  }

  throw_cnt_tz3++;
  delay(4000);
}

