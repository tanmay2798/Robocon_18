// Library
#include <SPI.h>
#include <PS3USB.h>
#include <Kangaroo.h>
//////////////////////// PS3 Variable /////////////////////////////
int lhx, lhy, rhx, rhy, l2;
int up = 0, down = 0;
int curup = 0, curdown = 0;
int ps3_deadband_trans = 25;
int ps3_deadband_rot = 15;

// for back motors.
KangarooSerial K(Serial1);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

// for front motor. 
KangarooSerial M(Serial2);
KangarooChannel M1(M, '2');
/////////////////////////System Variable //////////////////////////
float v1, v2, v3, vx, vy, w,vs,vxc,vyc;
int angle = 120;
int Serial_baud_rate = 19200;
int back_kangaroo_baud_rate = 19200;
int front_kangaroo_baud_rate = 19200;
int drive_kang_ramp = 40000;              // change later
float max_speed = 3000;                  // change later
int speed_factor = 15;
int speed_factor_initial = 7;
int speed_factor_final = 30;
int rot_divider=4.3;
boolean debug_enable = true;
int avg_speed;

void setup() {
  Serial1.begin(front_kangaroo_baud_rate);    // Sabertooth only
  Serial2.begin(back_kangaroo_baud_rate);
  Serial.begin(Serial_baud_rate);
  PS3_init();
  Serial.println("PS3 started");

  back_Kangaroo_init();
  Serial.println("back kangaroo started");
  front_kangaroo_init();
  Serial.println("front kangaroo started");

}

void loop() {
  PS3_getValue();
  PS3_deadband_trans(ps3_deadband_trans);
  PS3_deadband_rot(ps3_deadband_rot);
  
  if (lhx > 0){
    lhx-=ps3_deadband_trans;
  }else if (lhx < 0){
    lhx+=ps3_deadband_trans;
  }
  if (lhy > 0){
    lhy-=ps3_deadband_trans;
  }else if (lhy < 0){
    lhy+=ps3_deadband_trans;
  }
  if (rhy > 0){
    rhy-=ps3_deadband_rot;
  }else if (rhy < 0){
    rhy+=ps3_deadband_rot;
  }
  vx = lhx;
  vy = lhy;
  w = rhx;
  vxc=-0.5*K1.getS().value()/speed_factor-0.5*K2.getS().value()/speed_factor+M1.getS().value()/speed_factor;
  vyc=-0.866*K1.getS().value()/speed_factor+0.866*K2.getS().value()/speed_factor;
  avg_speed = (int)pow((vxc*vxc + vyc*vyc),0.5);
  vs=abs(vy);
  speed_factor = map(avg_speed, 0, 127*1.42, speed_factor_initial, speed_factor_final);
  v1 = -0.3333 * vx - 0.5774 * vy + 0.3333 * w/rot_divider;
  v2 = -0.3333 * vx + 0.5774 * vy + 0.3333 * w/rot_divider;
  v3 = 0.6667 * vx + 0.3333 * w/rot_divider;
  //v1=0.3333*vx-up*50+down*50+0.3333*w;
  //v2=0.3333*vx+up*50-down*50+0.3333*w;
  if (debug_enable) {
    Serial.print(lhx);
    Serial.print("\t");
    Serial.print(lhy);
    Serial.print("\t");
    Serial.print(rhy);
    Serial.print("\t");
    Serial.print(K1.getS().value());
    Serial.print("||");
    Serial.print(v1*speed_factor);
    Serial.print("\t");
    Serial.print(K2.getS().value());
    Serial.print("||");
    Serial.print(v2*speed_factor);
    Serial.print("\t");
    Serial.print(M1.getS().value());
    Serial.print("||");
    Serial.print(v3*speed_factor);
    Serial.print("---|");
    Serial.println(speed_factor);
    delay(100);
  }

  drive_kangaroo_update(v1, v2, v3, true);
}
