// Library
#include <SPI.h>
#include <PS3USB.h>
#include <Kangaroo.h>
//////////////////////// PS3 Variable /////////////////////////////
int lhx, lhy, rhx, rhy, l2, r2;
int up = 0, down = 0;
int curup = 0, curdown = 0;
int ps3_deadband = 25;

// for back motors.
KangarooSerial K(Serial1);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

// for front motor. 
KangarooSerial M(Serial2);
KangarooChannel M1(M, '2');
/////////////////////////System Variable //////////////////////////
float v1, v2, v3, vx, vy, w;
int angle = 120;
int Serial_baud_rate = 19200;
int back_kangaroo_baud_rate = 19200;
int front_kangaroo_baud_rate = 19200;
int drive_kang_ramp = 1700;              // change later
float max_speed = 3000;                  // change later
int speed_factor = 20;
boolean debug_enable = true;
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
  PS3_deadband(ps3_deadband);
  if (lhx > 0){
    lhx-=ps3_deadband;
  }else if (lhx < 0){
    lhx+=ps3_deadband;
  }
  if (lhy > 0){
    lhy-=ps3_deadband;
  }else if (lhy < 0){
    lhy+=ps3_deadband;
  }
  if (rhx > 0){
    rhx-=ps3_deadband;
  }else if (lhy < 0){
    rhx+=ps3_deadband;
  }
  vx = lhx;
  vy = lhy;
  w = rhx;
  v1 = -0.3333 * vx - 0.5774 * vy + 0.3333 * w;
  v2 = -0.3333 * vx + 0.5774 * vy + 0.3333 * w;
  v3 = 0.6667 * vx + 0.3333 * w;
  //v1=0.3333*vx-up*50+down*50+0.3333*w;
  //v2=0.3333*vx+up*50-down*50+0.3333*w;
  if (debug_enable) {
    Serial.print(lhx);
    Serial.print("\t");
    Serial.print(lhy);
    Serial.print("\t");
    Serial.print(rhx);
    Serial.print("\t");
    Serial.print(K1.getP().value());
    Serial.print("||");
    Serial.print(v1);
    Serial.print("\t");
    Serial.print(K2.getP().value());
    Serial.print("||");
    Serial.print(v2);
    Serial.print("\t");
    Serial.print(M1.getP().value());
    Serial.print("||");
    Serial.println(v3);
  }

  drive_kangaroo_update(v1, v2, v3, true);
}
