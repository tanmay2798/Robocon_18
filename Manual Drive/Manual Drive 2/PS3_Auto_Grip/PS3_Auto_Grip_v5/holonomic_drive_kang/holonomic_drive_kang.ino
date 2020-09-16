// Library
#include <SPI.h>
#include <PS3USB.h>
#include <Kangaroo.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
///////////// Kangaroo /////////////////
int back_kangaroo_baud_rate = 19200;
int front_kangaroo_baud_rate = 19200;

KangarooSerial K(Serial1);          // for back motors.
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');
KangarooSerial M(Serial2);          // for front motor. 
KangarooChannel M1(M, '2');


//////////////////////// PS3 Variable /////////////////////////////
int ps3_deadband_trans = 25;
int ps3_deadband_rot_analog = 15;
int ps3_deadband_rot_lr = 20;
float rot_multiplier_analog=0.4;
float rot_multiplier_lr=1;

float lhx, lhy, rhx, rhy;

/////////////////////////System Variable //////////////////////////
int Serial_baud_rate = 19200;
int speed_factor = 15;
int speed_factor_initial = 15;
int speed_factor_final = 15;
boolean debug_enable = true;
int drive_kang_ramp = 40000;              // change later
float max_speed = 3000;                  // change later

float v1, v2, v3, vx, vy, w,vs;
int angle = 120;
int avg_speed;
int target_x;
int target_y;

///////////////////// Manual Ramping //////////////////////////////
double ramp_factor=75;       // step_acc * speed_factor
double step_acc = ramp_factor/speed_factor;
double step_acc_rot = 7;

int omega = 0;
int vel_x = 0;
int vel_y = 0;
boolean motion_stop = false;

//////////////////// Automation Variables ////////////////////////
Encoder Enc(2,3);

boolean automation=false;
int automation_state = 0;
long dis_origin = 0;
long enc_dis;
long dis_start_to_load = 18000;
long dis_start_to_load_slow = 6500;
long dis_load_to_tz1 = 54000;
long dis_load_to_tz1_slow = 18000;
int dis_transfer;
int dis_stop;
int max_speed_1=100;
int max_speed_2=180;
int slow_speed_1=20;
int slow_speed_2=20;
int auto_x = 0;
int auto_y = 0;

/////////////////////Rack Variable/////////////////////////////
int rack_1_pin = 9;
int rack_2_pin = 8;
bool rack_1_state = false;
bool rack_2_state = false;

Encoder rack_enc(20,21);
SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified rack_st(SWSerial);
//int rack_kangaroo_baud_rate = 19200;
//KangarooSerial Rack(Serial3);          // for rack.
//KangarooChannel Rack1(Rack, '1');
bool rack_rotate = false;
long cur_rack_pos = 0;
long prev_rack_pos = 0;
int rack_w = 50;
long rack_angle = 300;

void setup() {
   rack_init();
  digitalWrite(rack_1_pin,LOW);
  digitalWrite(rack_2_pin,LOW);
  Serial1.begin(front_kangaroo_baud_rate);    // Sabertooth only
  Serial2.begin(back_kangaroo_baud_rate);
  Serial.begin(Serial_baud_rate);
  PS3_init();
  Serial.println("PS3 started");
  front_kangaroo_init();
  Serial.println("front kangaroo started");
  back_Kangaroo_init();
  Serial.println("back kangaroo started");
  SWSerial.begin(9600);
//  rack_init();
//  Serial.println("Rack kangaroo started");

  pinMode(13, OUTPUT);
}

void loop() {
  PS3_getValue();                 // Get Value from PS3
  get_encoder_val();              // Get Free Wheel Encoder Value
  automatic_motion();             // Overwrite/Add automatic velocities
  rack_1_rotate();
  target_x = auto_x + lhx;
  target_y = auto_y + lhy;
  ramp();                         // Ramp velocities
  if (motion_stop){               // Motion Disable
    vel_x = 0;
    vel_y = 0;
    omega = 0;
    automation_state = 0;
  }
  //----------- Get velocities of wheels from vel_x and vel_y --------------
  vx = vel_x;
  vy = vel_y;
  w = -omega;
  avg_speed = (int)pow((vx*vx + vy*vy),0.5);
  vs=abs(avg_speed);
  speed_factor = map(avg_speed, 0, (127-ps3_deadband_trans)*1.42, speed_factor_initial, speed_factor_final);
  v1 = -0.3333 * vx - 0.5774 * vy + 0.3333 * w;
  v2 = -0.3333 * vx + 0.5774 * vy + 0.3333 * w;
  v3 = 0.6667 * vx + 0.3333 * w;
  //-----------------------------------------------------------
  drive_kangaroo_update(v1, v2, v3, true);        // Write value to kangaroo
  rack_1_grip();                                  // Normal rack Gripping
  rack_2_grip();                                  // Golden rack gripping
  if (debug_enable) {
    Serial.print(Enc.read());
    Serial.print(" Enc_dis ");
    Serial.print(enc_dis);
    Serial.print(" Tar_X ");
    Serial.print(target_x);
    Serial.print(" Actual_X ");
    Serial.print(vel_x);
    Serial.print(" Tar_Y ");
    Serial.print(target_y);
    Serial.print(" Actual_Y ");
    Serial.print(vel_y);
    Serial.print(" Tar_w ");
    Serial.print(rhy);
    Serial.print(" Actual_w ");
    Serial.println(omega);
//    Serial.print("\t");
//    Serial.print(K1.getS().value());
//    Serial.print("||");
//    Serial.print(v1*speed_factor);
//    Serial.print("\t");
//    Serial.print(K2.getS().value());
//    Serial.print("||");
//    Serial.print(v2*speed_factor);
//    Serial.print("\t");
//    Serial.print(M1.getS().value());
//    Serial.print("||");
//    Serial.print(v3*speed_factor);
//    Serial.print("---|");
//    Serial.println(speed_factor);
  }
}

void ramp(){
  if (target_x >= vel_x){
    if ((target_x - vel_x) > step_acc){
      vel_x += step_acc;
    }else{
      vel_x = target_x;
    }
  }else if (target_x < vel_x){
    if ((vel_x - target_x) > step_acc){
      vel_x -= step_acc;
    }else{
      vel_x = target_x;
    }
  }
  if (target_y >= vel_y){
    if ((target_y - vel_y) > step_acc){
      vel_y += step_acc;
    }else{
      vel_y = target_y;
    }
  }else if (target_y < vel_y){
    if ((vel_y - target_y) > step_acc){
      vel_y -= step_acc;
    }else{
      vel_y = target_y;
    }
  }
  if (rhx >= omega){
    if ((rhx - omega) > step_acc_rot){
      omega += step_acc_rot;
    }else{
      omega = rhx;
    }
  }else if (rhx < omega){
    if ((omega - rhx) > step_acc_rot){
      omega -= step_acc_rot;
    }else{
      omega = rhx;
    }
  }
}

