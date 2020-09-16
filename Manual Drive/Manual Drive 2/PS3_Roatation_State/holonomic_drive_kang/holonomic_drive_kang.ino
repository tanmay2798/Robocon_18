// Library
#include <SPI.h>
#include <PS3USB.h>
#include <Kangaroo.h>
//////////////////////// PS3 Variable /////////////////////////////
float lhx, lhy, rhx, rhy, l2;

int ps3_deadband_trans = 25;
int ps3_deadband_rot_analog = 15;
int ps3_deadband_rot_lr = 20;
float rot_multiplier_analog=0.4;
float rot_multiplier_lr=1;

// for back motors.
KangarooSerial K(Serial1);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

// for front motor. 
KangarooSerial M(Serial2);
KangarooChannel M1(M, '2');
/////////////////////////System Variable //////////////////////////
int state=0;
float v1, v2, v3, vx, vy, w,vs;
int angle = 120;
int Serial_baud_rate = 19200;
int back_kangaroo_baud_rate = 19200;
int front_kangaroo_baud_rate = 19200;
int speed_factor = 15;
boolean debug_enable = true;
int avg_speed;

int drive_kang_ramp = 40000;              // change later

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
  
  vx = -lhx;
  vy = -lhy;
  w = rhx;
  avg_speed = (int)pow((vx*vx + vy*vy),0.5);
  vs=abs(avg_speed);
  speed_factor = map(avg_speed, 0, (127-ps3_deadband_trans)*1.42, speed_factor_initial, speed_factor_final);
  if (state==0){
    v1 = -0.5774 * vx - 0.3333 * vy + 0.3333 * w;
    v2 = 0.5774 * vx -0.3333 * vy + 0.3333 * w;
    v3 = 0.6667 * vy + 0.3333 * w;
  }
  else if (state==1){
    v1 = -0.3333 * vx - 0.5774 * vy + 0.3333 * w;
    v2 = -0.3333 * vx + 0.5774 * vy + 0.3333 * w;
    v3 = 0.6667 * vx + 0.3333 * w;
  }
  else if (state==2){
    v1 =-( -0.5774 * vx - 0.3333 * vy + 0.3333 * w);
    v2 = -(0.5774 * vx -0.3333 * vy + 0.3333 * w);
    v3 = -(0.6667 * vy + 0.3333 * w); 
  }
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
  }
  drive_kangaroo_update(v1, v2, v3, true);
}
