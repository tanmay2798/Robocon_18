// Library
#include <SPI.h>
#include <Kangaroo.h>
#include <Encoder.h>
#include <SabertoothSimplified.h>

///////////// Kangaroo /////////////////
boolean kang_enable = true;
int two_motor_kang_baud_rate = 19200;
int one_motor_kang_baud_rate = 19200;
int drive_kang_ramp = 40000;        // kang ramp

KangarooSerial K(Serial2);          // for two motor kang.
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');
KangarooSerial M(Serial3);          // for one motor kang. 
KangarooChannel M1(M, '2');

//////////////////////// PS3 Variable /////////////////////////////
boolean PS3_enable = true;
int ps3_deadband_trans = 25;
int ps3_deadband_rot_analog = 15;
int ps3_deadband_rot_lr = 20;
float rot_multiplier_analog=0.4;
float rot_multiplier_lr=1;

float ps_x, ps_y, ps_w;

/////////////////////////System Variable //////////////////////////
boolean speed_factor_change = false;
boolean debug_enable = true;
int Serial_baud_rate = 19200;
int speed_factor = 15;
int speed_factor_initial = 8;
int speed_factor_final = 15;

float v1, v2, v3, vx, vy, w,vs;
int avg_speed;
int target_x;
int target_y;
boolean update_state;
///////////////////// Manual Ramping //////////////////////////////
double ramp_factor_acc=75;                            // step_acc * speed_factor
double ramp_factor_deacc=140;                            // step_acc * speed_factor
double step_acc = ramp_factor_acc/speed_factor;
double step_deacc = ramp_factor_deacc/speed_factor;
double step_acc_rot = 7;
double step_deacc_rot = 10;

int omega = 0;
int vel_x = 0;
int vel_y = 0;
boolean motion_stop = false;

//////////////////// Automation Variables ////////////////////////
long dis_start_to_load = 13800;
long dis_start_to_load_slow = 2500;
long dis_load_to_tz1 = 40000;
long dis_load_to_tz1_slow = 18000;
int dis_transfer;
int dis_stop;

int max_speed_1=100;
int max_speed_2=180;
int slow_speed_1=20;
int slow_speed_2=20;

Encoder Enc(20,21);
boolean automation=false;
long enc_dis;
int auto_x = 0;
int auto_y = 0;
int automation_state = 0;
long dis_origin = 0;

/////////////////////Rack Variable/////////////////////////////
int rack_w = 50;
int rack_baud_rate = 9600;
int rack_1_pin_grip = 30;
int rack_1_pin_lift = 32;
int rack_2_pin = 34;
bool rack_1_state = false;
bool rack_2_state = false;
int rack_rotate = 0;

Encoder y_enc(2,3);
SabertoothSimplified rack_st(Serial1);
long cur_rack_pos = 0;
long prev_rack_pos = 0;
long rack_angle = 300;

void setup() {
  rack_init();
  if (kang_enable){
    Serial3.begin(one_motor_kang_baud_rate);    // Sabertooth only
    Serial2.begin(two_motor_kang_baud_rate);
    one_motor_kangaroo_init();
    Serial.println("One motor kang started");
    two_motor_kangaroo_init();
    Serial.println("Two motor kang started");
  }
  Serial1.begin(rack_baud_rate);            // Rack Motor
  Serial.begin(Serial_baud_rate);
  if (PS3_enable){
    PS3_init();
    Serial.println("PS3 Initialized");
  }
  pinMode(13,OUTPUT);
}

void loop() {
  if (PS3_enable){
    PS3_getValue();                 // Get Value from PS3
  }
  if (update_state){
    get_encoder_val();              // Get Free Wheel Encoder Value
    automatic_motion();             // Add automatic velocities
    target_x = auto_x + ps_x;       // Target velocities
    target_y = auto_y + ps_y;
    ramp();                         // Ramp velocities target_x ---> vel_x
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
    if (speed_factor_change){                             // Reduce speed and ramp factor for fine manual control
      speed_factor_initial = 5;
      speed_factor_final = 10;
      ramp_factor_acc = 50;
    }
    else{
      speed_factor_initial = 8;
      speed_factor_final = 15;
      ramp_factor_acc = 75;
    }
    avg_speed = (int)pow((vx*vx + vy*vy),0.5);
    speed_factor = map(avg_speed, 0, (127-ps3_deadband_trans)*1.42, speed_factor_initial, speed_factor_final);
    step_acc = ramp_factor_acc/speed_factor;
    v1 = -(0.3333 * vx - 0.5774 * vy - 0.3333 * w);
    v2 = (-0.3333 * vx - 0.5774 * vy + 0.3333 * w);
    v3 = 0.6667 * vx + 0.3333 * w;
    //-----------------------------------------------------------
    if (kang_enable){
      drive_kangaroo_update(v1, v2, v3, true);        // Write value to kangaroo
    }
    rack_1_rotate();                                // Check and rotate rack
//    rack_1_grip();                                 // Normal rack Gripping: Check and do
//    rack_2_grip();                                 // Golden rack gripping: Check and do
  }
  
  if (debug_enable){
    Serial.print(" Enc_dis ");
    Serial.print(enc_dis);
    Serial.print(" y_dis ");
    Serial.print(y_enc.read());
    Serial.print(" Tar_X ");
    Serial.print(target_x);
    Serial.print(" Actual_X ");
    Serial.print(vel_x);
    Serial.print(" Tar_Y ");
    Serial.print(target_y);
    Serial.print(" Actual_Y ");
    Serial.print(vel_y);
    Serial.print(" Tar_w ");
    Serial.print(ps_w);
    Serial.print(" Actual_w ");
    Serial.println(omega);
  }
}

void ramp(){
  if (target_x >= vel_x){
    if (vel_x >= 0){
      if ((target_x - vel_x) > step_acc){
        vel_x += step_acc;
      }
      else{
        vel_x = target_x;
      }
    }else{
      if ((target_x - vel_x) > step_deacc){
        vel_x += step_deacc;
      }
      else{
        vel_x = target_x;
      }
    }
  }
  else if (target_x < vel_x){
    if (vel_x < 0){
      if ((vel_x - target_x) > step_acc){
        vel_x -= step_acc;
      }
      else{
        vel_x = target_x;
      }
    }
    else{
      if ((vel_x - target_x) > step_deacc){
        vel_x -= step_deacc;
      }
      else{
        vel_x = target_x;
      }
    }
  }
  if (target_y >= vel_y){
    if (vel_y >= 0){
      if ((target_y - vel_y) > step_acc){
        vel_y += step_acc;
      }
      else{
        vel_y = target_y;
      }
    }
    else{
      if ((target_y - vel_y) > step_deacc){
        vel_y += step_deacc;
      }
      else{
        vel_y = target_y;
      }
    }
  }
  else if (target_y < vel_y){
    if (vel_y < 0){
      if ((vel_y - target_y) > step_acc){
        vel_y -= step_acc;
      }
      else{
        vel_y = target_y;
      }
    }
    else{
      if ((vel_y - target_y) > step_deacc){
        vel_y -= step_deacc;
      }
      else{
        vel_y = target_y;
      }
    }
  }
  if (ps_w >= omega){
    if (omega >= 0){
      if ((ps_w - omega) > step_acc_rot){
        omega += step_acc_rot;
      }
      else{
        omega = ps_w;
      }
    }
    else{
      if ((ps_w - omega) > step_deacc_rot){
        omega += step_deacc_rot;
      }
      else{
        omega = ps_w;
      }
    }
  }
  else if (ps_w < omega){
    if (omega < 0){
      if ((omega - ps_w) > step_acc_rot){
        omega -= step_acc_rot;
      }
      else{
        omega = ps_w;
      }
    }
    else{
      if ((omega - ps_w) > step_deacc_rot){
        omega -= step_deacc_rot;
      }
      else{
        omega = ps_w;
      }
    }
  }
}

